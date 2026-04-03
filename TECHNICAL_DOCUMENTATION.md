Markdown

# RTScheduler-Discovery: Technical Deep Dive

## Core Scheduler Architecture

The RTScheduler-Discovery is a production-grade, real-time preemptive scheduler designed for ARM Cortex-M microcontrollers. It implements a sophisticated 5-level priority scheduling system with event-driven I/O management.

### Scheduler Fundamentals

**Priority Levels:** 5 levels (0-4) + 1 blocked priority (5)
- Priority 0 = Highest priority (critical real-time tasks)
- Priority 4 = Lowest priority (background tasks)
- Priority 5 = Blocked (used internally for suspended tasks)

**Task States:**

IDLE (0) → RUNNING (1) → READY (2) ↓ ↑ └─ TIMER (3) ─┘ └─ IO (4) ────┘ → DEAD (6)
Code


### Scheduling Algorithm

**SysTick Handler (sys.c:97-185):**
1. Processes timer queue every 10ms
2. Moves expired timer-blocked tasks to READY
3. Evaluates priority preemption
4. Implements time-slicing for same-priority tasks
5. Performs task garbage collection

**Context Switching:**
- Preemptive: Higher priority task immediately switches in
- Time-sliced: Same priority tasks get 10ms quantum (configurable)
- Non-preemptive: Lower priority tasks run until blocking

## Task Management System

### Task Control Block (TCB)

```c
struct task {
    char *name;                    // Task identifier string
    void *sp;                      // Stack pointer (current execution point)
    int id;                        // Unique task ID
    struct task *next;             // Link in ready queue
    struct task *next2;            // Link in all-tasks chain
    int state;                     // Current state (READY, RUNNING, TIMER, IO, etc.)
    int prio_flags;                // Priority (0-4) + flags (TMARK for time-slice)
    void *estack;                  // End of allocated stack (for overflow detection)
    struct user_fd *fd_list;       // Linked list of open file descriptors
    unsigned int active_tics;      // Total execution ticks
    struct address_space *asp;     // Address space (for MMU support)
    int sel_data_valid;            // Select operation state cache
    struct sel_data sel_data;      // Cached ready FDs from select()
    struct blocker blocker;        // Blocking structure for sleep/I/O wait
};

Task Lifecycle

1. Creation (SVC_CREATE_TASK)
Code

Allocate 4KB page → Initialize TCB → Setup execution stack → 
Configure return context → Add to task list → Place in READY queue

2. Execution
Code

Task runs until:
  - Calls I/O operation (blocks on I/O)
  - Calls sleep() (blocks on timer)
  - Higher priority task ready (preempted)
  - Time quantum expires (time-sliced)

3. Blocking
Code

Task blocked → Moved to wait queue (IO or TIMER) → 
CPU time allocated to next ready task

4. Wakeup
Code

Driver callback or timer → sys_wakeup() called → 
Task moved to READY queue → May preempt current task

5. Termination
Code

Task calls SVC_DESTROY_SELF → Cleanup resources → 
Move to cemetery → Garbage collection on next tick

Priority Management

Dynamic Priority Adjustment (SVC_SETPRIO_TASK):
C

// Get current priority
int curr_prio = GET_PRIO(task->prio_flags);

// Set new priority (0-4)
SET_PRIO(task, new_prio);

// If downgrading priority of current task, reschedule
if (curr_prio < new_prio) {
    switch_on_return();  // Let higher priority task run
}

Priority Inversion Prevention:

    Time-slicing SET_TMARK prevents single task hogging CPU
    Blocking moves task out of ready queue
    Interrupt handlers use blocker callbacks to wake appropriate priority

I/O Subsystem: Event-Driven Architecture
Non-Blocking I/O Model

The I/O subsystem uses a completely asynchronous, event-driven model:

    No task spins waiting for data
    Drivers interrupt on data ready
    Tasks automatically block/unblock via callbacks

File Descriptor Management

Global FD Table:
C

#define FDTAB_SIZE 16
struct user_fd fd_tab[FDTAB_SIZE];  // FD 0 reserved for console

Per-Task FD Lists:
C

struct task {
    struct user_fd *fd_list;  // Linked list of this task's open FDs
};

struct user_fd {
    struct driver *driver;         // Pointer to driver instance
    struct device_handle *dev_handle;  // Device-specific handle
    unsigned int flags;            // O_NONBLOCK, etc.
    struct user_fd *next;          // Next FD in task's list
};

Driver Interface
C

struct driver_ops {
    // Open device, set callback for events
    device_handle *(*open)(void *driver_instance, DRV_CBH drv_cb, void *usr_ref);
    
    // Close device
    int (*close)(struct device_handle *);
    
    // Device-specific control operations
    // Commands: RD_CHAR, WR_CHAR, IO_POLL, IO_CALL, F_SETFL, etc.
    int (*control)(struct device_handle *, int cmd, void *, int len);
    
    // Initialize driver (called once at startup)
    int (*init)(void *driver_instance);
    
    // Start driver (called when scheduler starts)
    int (*start)(void *driver_instance);
    
    // Update clock frequency (for baud rate adjustment, etc.)
    int (*clk_update)(void *driver_instance, int hz);
};

I/O Operation Workflow

Read Operation (io_read):
Code

1. Call driver->control(dh, RD_CHAR, buffer, size)
2. Driver returns:
   - DRV_OK + bytes read     → Return immediately
   - DRV_AGAIN               → sys_sleepon() task on blocker
   - DRV_ERR                 → Return error
3. Driver receives interrupt → calls sys_drv_wakeup()
4. Task moves to READY → Next tick executes
5. Retry io_read() → Data now available

Write Operation (io_write):
Code

1. Call driver->control(dh, WR_CHAR, buffer, size)
2. Driver returns:
   - DRV_OK + bytes sent     → Continue
   - DRV_AGAIN               → sys_sleepon() task
   - DRV_INPROGRESS          → Wait for completion callback
3. If partial write, loop until all bytes sent

Select/Multiplexing (io_select):
Code

1. Task provides read/write/state FD bitmasks and timeout
2. Scheduler checks each FD for readiness
3. If any ready, return immediately
4. Otherwise, task blocks on all FDs
5. When any FD ready, sys_drv_wakeup() wakes task
6. Returns set of ready FDs

Event Masking
C

#define EV_READ   1    // Device has data to read
#define EV_WRITE  2    // Device ready for writing
#define EV_STATE  4    // Device state changed (status event)

Callback Signature:
C

typedef int (*DRV_CBH)(struct device_handle *, int event, void *user_ref);

// Driver calls when event occurs:
// callback(dh, EV_READ, current_task);  // Data ready
// callback(dh, EV_WRITE, current_task); // Ready to write
// callback(dh, EV_STATE, current_task); // Status change

Driver Return Codes
C

#define DRV_OK          0   // Operation completed successfully
#define DRV_ERR         1   // Operation failed (error condition)
#define DRV_AGAIN      11   // Resource not ready, please retry
#define DRV_INPROGRESS 12   // Async operation in progress, wait for callback

System Call Interface (19 SVCs)
Task Management SVCs

SVC_CREATE_TASK (1)
C

struct task_create_args {
    void *fnc;              // Task function pointer
    void *val;              // Argument to pass
    unsigned int val_size;  // Size of argument
    int prio;               // Priority 0-4
    char *name;             // Task name string
};
// Returns: 0 on success, -1 on failure

SVC_SLEEP (2)
Code

// Sleep current task for milliseconds
// Arguments: timeout in ms
// Returns: remaining ms (if interrupted)

SVC_SLEEP_ON (3)
Code

// Sleep on blocker structure with optional timeout
// Used internally for I/O and timer waits

SVC_WAKEUP (4)
Code

// Wake specific blocker
// Used internally by sys_drv_wakeup()

SVC_DESTROY_SELF (6)
Code

// Current task terminates itself
// Returns: never (task switched out)

SVC_BLOCK_TASK (13)
Code

// Suspend task by name (doesn't run)
// Arguments: task name string
// Returns: 0 on success, -1 if task not found

SVC_UNBLOCK_TASK (14)
Code

// Resume task by name
// Arguments: task name string
// Returns: 0 on success, -1 if task not found

SVC_SETPRIO_TASK (15)
Code

// Change task priority dynamically
// Arguments: task name, new priority (0-4)
// Returns: 0 on success

I/O SVCs

SVC_IO_OPEN (5)
Code

// Open driver by name
// Arguments: driver name string
// Returns: file descriptor (positive) or -1

SVC_IO_READ (6)
Code

// Blocking read from file descriptor
// Arguments: fd, buffer, size
// Returns: bytes read or error code

SVC_IO_WRITE (7)
Code

// Blocking write to file descriptor
// Arguments: fd, buffer, size
// Returns: bytes written or error code

SVC_IO_CONTROL (8)
Code

// Device-specific control operation
// Arguments: fd, command, arg1, arg2
// Returns: command-specific value

SVC_IO_CLOSE (10)
Code

// Close file descriptor
// Arguments: fd
// Returns: 0 on success

SVC_IO_SELECT (11)
Code

// Multiplexed I/O on multiple FDs
// Arguments: fd_count, read_set, write_set, state_set, timeout
// Returns: number of ready FDs

SVC_IO_LSEEK (9)
Code

// Seek in device/file
// Arguments: fd, offset, whence (SEEK_SET/CUR/END)
// Returns: new position

SVC_IO_MMAP (12)
Code

// Memory-map device memory
// Arguments: addr, length, prot, flags, fd, offset
// Returns: virtual address or 0

System SVCs

SVC_GETTIC (17)
Code

// Get current system tick count (10ms units)
// Returns: tick count (unsigned int)

SVC_SETDEBUG_LEVEL (16)
Code

// Set debug output verbosity (if DEBUG build)
// Arguments: debug level
// Returns: 0 if DEBUG enabled, -1 if disabled

SVC_REBOOT (18)
Code

// Reboot system (requires magic cookie)
// Arguments: 0x5a5aa5a5 (magic value)
// Returns: -1 (or reboots)

SVC_SBRK (19)
Code

// Dynamic memory allocation (heap)
// Arguments: increment in bytes
// Returns: old break point (if MMU enabled)

SVC_BRK (20)
Code

// Set heap break
// Arguments: new break address
// Returns: 0 on success, -1 on failure

Blocking Mechanism: Task Synchronization
Blocker Structure
C

struct blocker {
    struct blocker *next;          // Timer queue link (circular)
    struct blocker *next2;         // Event list link (for select)
    unsigned int ev;               // Event mask + flags
    struct device_handle *dh;      // Associated device
    struct driver *driver;         // Associated driver
    unsigned int wake;             // Wakeup flag (set by driver)
    unsigned int wakeup_tic;       // Absolute tick for timer wakeup
};

Timer Queue (Circular Array)
C

#define TQ_SIZE 1024  // Circular timer queue
struct tq {
    struct blocker *tq_out_first;
    struct blocker *tq_out_last;
} tq[TQ_SIZE];

// Timer wheel:
// tq[(current_tick + offset) % TQ_SIZE]
// 10ms granularity: each bucket covers 10240ms window

Sleep Flow

Task sleeps (sys_sleepon):
Code

1. Task provides blocker and optional timeout
2. If already woken (wake flag set), return immediately
3. If device ready (device_ready returns true), return immediately
4. Otherwise:
   - Add blocker to timer queue at (current_tick + ms/10)
   - Change task state to TASK_STATE_TIMER
   - Call switch_now() to switch out task
5. When timer expires or device ready:
   - SysTick moves blocker to READY queue
   - Task restored to READY state
   - Next scheduling cycle runs task
6. Task resumes from sys_sleepon() call

Wake from I/O:
Code

1. Driver receives hardware interrupt
2. Calls registered callback: sys_drv_wakeup(dh, ev, task)
3. sys_drv_wakeup():
   - Sets blocker->wake = 1
   - Moves blocker from timer queue (if present)
   - If task has select() active, updates ready FD sets
   - Calls sys_wakeup() to move task to READY
4. If higher priority, calls switch_on_return()
5. Task runs on next scheduling opportunity

Memory Layout and Management
Per-Task Memory Allocation

4KB Page Layout:
Code

Offset   Size      Contents
------   ----      --------
0x000    0x100     struct task (TCB)
0x100    0x700     Reserved/unused
0x800    0x800     Code/data section (2KB)
0x1000   0x1000    Execution stack (2KB, grows downward)

Stack Setup:
C

// Task allocated at 4KB boundary
struct task *t = (struct task *)get_page();

// Stack grows downward from top of page
unsigned long int *stackp = (unsigned long int *)t + 4096;

// Setup context for first execution
setup_return_stack(t, stackp, task_function, 
                   cleanup_function, arg0, arg1);

Memory Protection (Optional MMU Support)
C

#ifdef MMU
struct address_space {
    unsigned long int *pgd;        // Page directory (1024 PTEs)
    int id;                        // Address space ID
    unsigned long int brk;         // Heap break
    unsigned long int mmap_vaddr;  // Last mmap address
    int ref;                       // Reference count
};

// Stack guard pages prevent overflow
// Each task has own address space
// Dynamic memory via sbrk()/brk()
#endif

Design Patterns
1. Container-Of Pattern

Problem: Extract containing structure from embedded member

Solution:
C

#define container_of(ptr, type, member) ({ \
    const typeof(((type *)0)->member) *__mptr = (ptr); \
    (type *)((char *)__mptr - offsetof(type, member)); })

// Usage: Extract task from its blocker
struct task *t = container_of(&task->blocker, struct task, blocker);

Benefits: Type-safe, zero-cost abstraction
2. Driver Abstraction

Problem: Support multiple hardware devices with different drivers

Solution:
C

// Unified driver interface
struct driver {
    char *name;                // "uart0", "spi1", etc.
    void *instance;            // Driver-specific data
    struct driver_ops *ops;    // Function pointers
    unsigned int stat;         // Init/started flags
    struct driver *next;       // Linked list
};

// Tasks open by name, don't need to know implementation
int fd = io_open("uart0");  // Returns generic FD

Benefits: Pluggable drivers, late binding, easy to test
3. Event-Driven Callbacks

Problem: Multiple tasks waiting for different events on different devices

Solution:
C

// Driver calls task callback when event occurs
typedef int (*DRV_CBH)(struct device_handle *, int event, void *user_ref);

// sys_drv_wakeup() can handle:
// - Single task waking (I/O)
// - Multiple tasks (select)
// - Event masking (only wake on specific event)

Benefits: No polling, CPU-efficient, deterministic latency
4. Interrupt-Safe Operations

Problem: Data consistency during interrupt handlers

Solution:
C

unsigned long int cpu_flags = disable_interrupts();
{
    // Critical section - no interrupts
    // Modify shared data structures safely
}
restore_cpu_flags(cpu_flags);

Benefits: Simple, deterministic, easy to analyze for correctness
5. Time-Slicing with TMARK

Problem: Single high-priority task monopolizing CPU

Solution:
C

#define GET_TMARK(task)  ((task)->prio_flags & 0x10)
#define SET_TMARK(task)  ((task)->prio_flags |= 0x10)
#define CLR_TMARK(task)  ((task)->prio_flags &= ~0x10)

// SysTick enforces time-slice:
if (GET_TMARK(current) && ready[GET_PRIO(current)]) {
    // Time slice expired, switch to next task at same priority
    switch_on_return();
}

Benefits: Fair CPU allocation, prevents priority inversion
Supported Boards
BLACKPILL (STM32F4)

    Used By: GM-OBD1 project
    CPU: STM32F407 @ 168 MHz (ARM Cortex-M4)
    Memory: 192 KB RAM, 512 KB Flash
    Peripherals: 6× USART, 3× SPI, 3× I2C, USB OTG
    Status: Production-tested

MB1075B (STM32L496)

    CPU: STM32L496 @ 80 MHz (ARM Cortex-M4, low-power)
    Memory: 128 KB RAM, 1 MB Flash
    Peripherals: Multiple UARTs, I2C, SPI
    Status: Supported

MB997C (STM32F407VE)

    CPU: STM32F407 @ 168 MHz (ARM Cortex-M4)
    Memory: 192 KB RAM, 512 KB Flash
    Peripherals: Extensive I/O support
    Status: Supported

GM-OBD1 Integration Example
Real-Time Automotive Diagnostic System

Multi-Threaded Task Architecture:
Code

Priority 3: ECU Protocol Thread
  ├─ Handles 160 baud ALDL communication
  ├─ Parses ECM diagnostic frames
  └─ Responds with data requests

Priority 2: Display Thread
  ├─ Updates scanner UI
  ├─ Formats and caches data
  └─ Manages user interface

Priority 1: USART Driver
  ├─ Hardware interrupt handler
  ├─ Calls sys_drv_wakeup() when data available
  └─ Low-level framing/buffering

Priority 0: Background
  └─ Monitoring, housekeeping

Communication Flow:
Code

1. ECM transmits 160 baud frame
   ↓
2. USART hardware interrupt fires
   ↓
3. Driver ISR buffers data, calls sys_drv_wakeup()
   ↓
4. sys_drv_wakeup() wakes protocol thread (priority 3)
   ↓
5. SysTick: Protocol thread moved to READY queue
   ↓
6. Display thread preempted (priority 2 < 3)
   ↓
7. Protocol thread runs (sub-10ms latency)
   ├─ Parse diagnostic frame
   ├─ Execute command
   └─ Schedule response write
   ↓
8. Protocol thread blocks on response I/O write
   ↓
9. Display thread resumes
   ├─ Updates cached values
   └─ Refreshes UI

Real-Time Guarantees:

    Protocol response latency: < 10ms (guaranteed by priority)
    No frame loss (driver buffers + DMA)
    UI updates smooth (time-sliced at priority 2)
    Deterministic behavior (no spinning/polling)

Summary

RTScheduler-Discovery combines elegant design patterns with practical real-time constraints:

    Preemptive Priority Scheduling ensures critical tasks run predictably
    Event-Driven I/O eliminates polling overhead
    Task Blocking allows efficient CPU utilization
    Driver Abstraction provides flexibility and testability
    Container-Of Pattern enables type-safe generic code
    Interrupt-Safe Operations guarantee data consistency

The result is a lightweight, deterministic RTOS suitable for embedded automotive, industrial control, and real-time applications on ARM Cortex-M microcontrollers.
