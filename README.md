# Transaction-Aware Lock Management

The implementation allow to concurrently lock and unlock items and block the execution thread until the lock 
could be acquired.
However, it should detect and handle deadlocks using the wait-for graph technique.
To block the thread, use a `std::shared_mutex`, and to abort a transaction, just throw an exception.

The `WaitsForGraph` should be able to add new waits-for dependencies, but abort the transaction as soon as the graph
becomes cyclic.
When a transaction finishes, it should also remove all of its waits-for dependants.
Keep in mind that the graph will need separate synchronization to the Lock Manager.

The `LockManager` should use a fixed-size hashtable with a lock for each bucket and chain.
To manage the lifetime of the lock objects themselves, it is recommended to use a [`std::shared_ptr`](https://en.cppreference.com/w/cpp/memory/shared_ptr/shared_ptr)
with a custom deleter that deletes the lock in the LockManager.
The `Lock` structure is already set to have its lifetime managed by a `std::shared_ptr<Lock>`.
You can read more about this [here](https://en.cppreference.com/w/cpp/memory/enable_shared_from_this).

## Waits For Graph

- DFS implementation: https://github.com/cakebytheoceanLuo/deadlock_detection_playground/tree/9bbeebe717d19cad82c364ab9119f31e6d569993

- Online cycle detection algorithm implementation: https://github.com/cakebytheoceanLuo/deadlock_detection_playground/tree/734c4926e8041b2c38f55a369e7b1011c7adb545
  - Follow the idea from: https://db.in.tum.de/teaching/ss20/moderndbs/chapter4.pdf?lang=de