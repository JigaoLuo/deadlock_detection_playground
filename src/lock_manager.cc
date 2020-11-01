// ---------------------------------------------------------------------------------------------------
#include "lock_manager.h"
// ---------------------------------------------------------------------------------------------------
#include <cassert>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <unordered_set>
#include <queue>
// ---------------------------------------------------------------------------------------------------
namespace deadlock_detection {
// ---------------------------------------------------------------------------------------------------
void Transaction::addLock(DataItem dataItem, LockMode mode) {
  assert(lockManager);
  auto lock = lockManager->acquireLock(*this, dataItem, mode);
  locks.push_back(std::move(lock));
}

Transaction::~Transaction() {
  if (lockManager) {
    lockManager->wfg.removeTransaction(*this);
  }

  for (auto& lock : locks) {
    {
      auto guard = std::unique_lock(lock->metadata_latch);
      lock->owners.erase(std::remove(lock->owners.begin(), lock->owners.end(), this), lock->owners.end());
    }
    if (lock->ownership == LockMode::Shared) {
      lock->lock.unlock_shared();
    } else {
      lock->lock.unlock();
    }
  }
}

using Node = std::pair<const Transaction* const, std::vector<const Transaction*>>;
using Graph = const std::unordered_map<const Transaction*, std::vector<const Transaction*>>;
using VisitedSet = std::unordered_set<const Transaction*>;

static bool findDuplicate(Graph& graph, const Node* node, const VisitedSet& visited) {
  if (visited.find(node->first) != visited.end()) {
    return true;
  }

  auto current_visited = visited;  /// A copy.
  current_visited.insert(node->first);
  for (const auto& transcation : node->second) {
    auto it = graph.find(transcation);
    if (it == graph.end()) {
      continue;
    }
    auto& n = *it;
    /// DFS.
    if (findDuplicate(graph, &n, current_visited)) {
      return true;
    }
  }
  return false;
}

static bool findDuplicate(Graph& graph, const Node* node) {
  /// DFS.
  auto visited = VisitedSet();
  return findDuplicate(graph, node, visited);
}

std::pair<int, int> WaitsForGraph::checkOrder(const Transaction* b, const Transaction* a) {
  int b_offset = -1;
  int a_offset = -1;
  for (size_t i = 0; i < topo_order.size(); ++i) {
    if (topo_order[i] == b) {
      b_offset = i;
    }
    if (topo_order[i] == a) {
      a_offset = i;
    }
  }
  assert(b_offset != -1 && a_offset != -1);
  return {b_offset, a_offset};
}

bool WaitsForGraph::dfs(const Transaction* a, const Transaction* b) {
  /// Dfs from a: Check if a->b (b is the start), then cycle return false.
  marker[a] = Mark::Visited;
  auto it = graph.find(a);
  if (it == graph.end()) return true;  /// No cycle.

  /// a->v1, ..., a->vn
  for (const auto& v : it->second) {
    auto [v_offset, b_offset] = checkOrder(v, b);
    if (v_offset <= b_offset) {
      if (marker[v] == Mark::Start) {
        return false;   /// Cycle.
      } else if (marker[v] == Mark::Unmarked) {
        if (!dfs(v, b)) {
          return false;  /// Cycle.
        }
      }
    }
    return true;  /// No cycle.
  }
}

bool WaitsForGraph::onlineEdgeCheck(const Transaction* b, const Transaction* a) {
  auto [b_offset, a_offset] = checkOrder(b, a);
  if (b_offset < a_offset) {
    return true;
  }

  /// Topo ordering: a < ... < b
  marker[b] = Mark::Start;
  if (!dfs(a, b)) {
    assert(a_offset <= b_offset);
    for (size_t i = a_offset; i <= b_offset; ++i) {
      marker[topo_order[i]] = Mark::Unmarked;
    }
    return false;  /// Cycle.
  }

  /// Shift: the goal to have  ... < b < a < (all nodes depending on a).
  marker[b] = Mark::Unmarked;
  size_t shift = 0;
  std::vector<const Transaction*> L;
  assert(a_offset <= b_offset);
  for (size_t i = a_offset; i <= b_offset; ++i) {
    if (marker[topo_order[i]] != Mark::Unmarked) {
      L.push_back(topo_order[i]);
      ++shift;
      marker[topo_order[i]] = Mark::Unmarked;
    } else {
      assert(i - shift >= 0);
      topo_order[i - shift] = topo_order[i];
    }
  }

  /// TODO(future): memcpy
  for (size_t i = 0; i < L.size(); ++i) {
    topo_order[b_offset - L.size() + 1] = L[i];
  }
  return true;
}

void WaitsForGraph::addWaitsFor(const Transaction &transaction, const Lock &lock) {
  auto gurad = std::unique_lock(mutex);

  /// Add edges: transaction waits for all owners of the lock.
  ///            transaction -> all owners of the lock.
  auto& waits_for = graph[&transaction];
  auto& mark = marker[&transaction];  /// Probably insert a new node with unmarked.
  for (auto* other_tx : lock.owners) {
    if (std::find(waits_for.begin(), waits_for.end(), other_tx) == waits_for.end()) {
      waits_for.push_back(other_tx);
      auto& mark = marker[other_tx];  /// Probably insert a new node with unmarked.
      if (topo_order_generated) {
        if (std::find(topo_order.begin(), topo_order.end(), other_tx) == topo_order.end()) {
          topo_order.push_back(other_tx);
        }
      }
    }
  }
  if (topo_order_generated) {
    if (std::find(topo_order.begin(), topo_order.end(), &transaction) == topo_order.end()) {
      topo_order.push_back(&transaction);
    }
  }

  /// Check if necessary to generate the topo ordering.
  if (topo_order.empty() && graph.size() > 1 && !topo_order_generated) {
    if (!generateTopologicalOrdering()) {
      graph.erase(&transaction);
      marker.erase(&transaction);
      topo_order.clear();
      throw DeadLockError();
    }
    topo_order_generated = true;  /// Ensure built without deadlock.
  } else
    if (graph.size() <= 1) {
    return;
  }

  /// The check for cycles: if the owner is waiting for transaction.
  /// Here assume the topo ordering exists.

  /// Check each newly added edges.
  for (auto* other_tx : lock.owners) {
    if (!onlineEdgeCheck(&transaction, other_tx)) {
      graph.erase(&transaction);
      topo_order.erase(std::remove(topo_order.begin(), topo_order.end(), &transaction), topo_order.end());
      marker.erase(&transaction);
      throw DeadLockError();
    }
  }
}

void WaitsForGraph::addWaiters(const Transaction& owner, const std::vector<const Transaction*>& waiters) {
  auto guard = std::unique_lock(mutex);

  /// Add edges.
  /// A edge: waiters -> owner.
  auto& mark = marker[&owner];  /// Probably insert a new node with unmarked.
  for (auto* waiter : waiters) {
    auto& already_waits_for = graph[waiter];
    if (std::find(already_waits_for.begin(), already_waits_for.end(), &owner) == already_waits_for.end()) {
      already_waits_for.push_back(&owner);
      auto& mark = marker[waiter];  /// Probably insert a new node with unmarked.
      if (topo_order_generated) {
        if (std::find(topo_order.begin(), topo_order.end(), waiter) == topo_order.end()) {
          topo_order.push_back(waiter);
        }
      }
    }
  }
  if (topo_order_generated) {
    if (std::find(topo_order.begin(), topo_order.end(), &owner) == topo_order.end()) {
      topo_order.push_back(&owner);
    }
  }

  /// The check for cycles: if the owner is waiting for transaction.
  auto outgoing = graph.find(&owner);
  if (outgoing == graph.end()) {
    return;
  }

  for (auto* waiter : waiters) {
    if (!onlineEdgeCheck(waiter, &owner)) {
      throw DeadLockError();
    }
  }
}

void WaitsForGraph::removeTransaction(const Transaction &transaction) {
  auto guard = std::unique_lock(mutex);

  graph.erase(&transaction);
  for (auto& list : graph) {
    auto& v = list.second;
    v.erase(std::remove(v.begin(), v.end(), &transaction), v.end());
  }
  topo_order.erase(std::remove(topo_order.begin(), topo_order.end(), &transaction), topo_order.end());
  marker.erase(&transaction);
}

bool WaitsForGraph::generateTopologicalOrdering() {
  /// Create a vector to store in-degrees of all vertices.
  /// Initialize all in-degrees as 0.
  std::unordered_map<const Transaction*, size_t> in_degree;
  in_degree.reserve(graph.size());

  /// Traverse adjacency lists to fill in-degrees of vertices.
  /// This step takes O(V+E) time.
  for (const auto& it : graph) {
    auto& counter = in_degree[it.first];  /// Ensure all V in the ht.
    for (const auto& in : it.second) {
      auto& counter = in_degree[in];
      ++counter;
    }
  }

  /// Create an queue and enqueue all vertices with in-degree 0.
  std::queue<const Transaction*> q;
  for (const auto& it : in_degree) {
    if (it.second == 0) {
      q.push(it.first);
    }
  }

  /// Initialize count of visited vertices.
  int cnt = 0;

  /// One by one dequeue vertices from queue and enqueue adjacents if in-degree of adjacent becomes 0.
  while (!q.empty()) {
    /// Extract front of queue (or perform dequeue) and add it to topological order.
    auto& u = q.front();
    q.pop();
    topo_order.push_back(u);

    /// Iterate through all its neighbouring nodes of dequeued node u and decrease their in-degree by 1.
    const auto& u_edge = graph[u];
    for (const auto& in : u_edge) {
      /// If in-degree becomes zero, add it to queue
      if (--in_degree[in] == 0) {
        q.push(in);
      }
    }
    cnt++;
  }

  /// Check if there was a cycle
  if (cnt != graph.size()) {
    return false;
  }
  return true;
//  /// Print topological order
//  for (int i = 0; i < topo_order.size(); i++)
//    std::cout << topo_order[i] << " ";
//  std::cout << std::endl;
}

LockManager::~LockManager() {
  for (auto& chain : table) {
    if (chain.first != nullptr) {
      Lock* prev_lock(nullptr);
      Lock* lock_ptr = chain.first;
      while (lock_ptr) {
        prev_lock = lock_ptr;
        lock_ptr = lock_ptr->next;
        delete(prev_lock);
      }
    }
  }
}

std::shared_ptr<Lock> LockManager::acquireLock(Transaction &transaction, DataItem dataItem, LockMode mode) {
  static constexpr auto hash = Hash();
  auto& chain = table[hash(dataItem) % table.size()];
  auto guard = std::unique_lock(chain.latch);

  auto lock = [&]() -> std::shared_ptr<Lock> {
    auto** list_head = &chain.first;
    while (*list_head) {
      /// Clean up expired locks.
      /// Lazy Deletion.
      if ((*list_head)->weak_from_this().expired()) {
        auto* expired_lock = *list_head;
        *list_head = expired_lock->next;
        delete expired_lock;
        continue;
      }

      /// If found the correct lock.
      if (**list_head == dataItem) {
        auto res = (*list_head)->weak_from_this().lock();  /// Get the shared_ptr.

        /// Possible, the shared pointer may expire concurrently.
        if (!res) {
          assert((*list_head)->weak_from_this().expired());
          auto* expired_lock = *list_head;
          *list_head = expired_lock->next;
          delete expired_lock;
        }

        return res;
      }
      list_head = &(*list_head)->next;
    }
    return nullptr;
  }();

  /// Lock not found OR lock expired.
  if (!lock) {
    lock = Lock::construct(dataItem);
    lock->next = chain.first;
    chain.first = lock.get();
  }

  /// Then the txn can release the lock on the chain and only lock the meta-latch.
  guard.unlock();
  auto lock_guard= std::unique_lock(lock->metadata_latch);

  /// Can the txn directly acquire the lock?
  if (mode == LockMode::Shared ? lock->lock.try_lock_shared() : lock->lock.try_lock()) {
    lock->ownership = mode;
    lock->owners.push_back(&transaction);
    if (!lock->waiters.empty()) {
      wfg.addWaiters(transaction, lock->waiters);
    }
    return lock;
  }

  /// If no, then the txn have to wait.
  wfg.addWaitsFor(transaction, *lock);
  lock->waiters.push_back(&transaction);
  lock_guard.unlock();

  if (mode == LockMode::Shared) {
    lock->lock.lock_shared();
  } else {
    lock->lock.lock();
  }

  /// Remove edge in wait-for graph.
  lock_guard.lock();
  lock->waiters.erase(std::remove(lock->waiters.begin(), lock->waiters.end(), &transaction), lock->waiters.end());

  lock->ownership = mode;
  lock->owners.push_back(&transaction);

  /// The txn get the lock, all the waits have to wait for the txn.
  if (!lock->waiters.empty()) {
    wfg.addWaiters(transaction, lock->waiters);
  }
  return lock;
}

LockMode LockManager::getLockMode(DataItem dataItem) {
  static constexpr auto hash = Hash();
  auto& chain = table[hash(dataItem) % table.size()];
  auto guard = std::unique_lock(chain.latch);

  auto* lock = [&]() -> Lock* {
    for (auto* it = chain.first; it; it = it->next) {
      if (*it == dataItem) {
        return it;
      }
    }
    return nullptr;
  }();
  if (!lock || lock->weak_from_this().expired()) {
    return LockMode::Unlocked;
  }
  return lock->ownership;
}

void LockManager::deleteLock(Lock *) {
  /// Intentionally left blank.
}
// ---------------------------------------------------------------------------------------------------
}  // namespace deadlock_detection
// ---------------------------------------------------------------------------------------------------
