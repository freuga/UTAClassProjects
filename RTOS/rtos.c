// RTOS Framework - Fall 2020
// J Losh

// Student Name:
// Freddy Aguinaga

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
#include "convert.h"
#include "mpu.h"

// RISC code
extern uint32_t getR0(void);
extern uint32_t getR1(void);
extern uint32_t getPSP(void);
extern uint32_t getMSP(void);
extern uint32_t getControl(void);
extern void pushR4toR11toPSP(void);
extern void popR4toR11fromPSP(void);
extern void setPSP(uint32_t add);
extern void setControl(uint32_t val);
extern void pushPC(void* add);
extern uint32_t getSVC(void);

// peripheral definitions
#define PRIVILEGE PORTE,5
#define RED_LED PORTE,1
#define ORANGE_LED PORTE,2
#define YELLOW_LED PORTE,3
#define GREEN_LED PORTE,4
#define BLUE_LED PORTF,2
#define BUTTON_0 PORTA,2
#define BUTTON_1 PORTA,3
#define BUTTON_2 PORTA,4
#define BUTTON_3 PORTA,5
#define BUTTON_4 PORTA,6
#define BUTTON_5 PORTA,7

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    char name[16];
    uint16_t count;
    uint16_t queueSize;
    uint8_t  lastTask;
    uint16_t oldestIndex;
    uint16_t newIndex;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;
uint8_t keyPressed, keyReleased, flashReq, resource;

#define KEYPRESSED 0
#define KEYRELEASED 1
#define FLASHREQ 2
#define RESOURCE 3
// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

bool piEnabled = false;
bool preemptEnabled = false;

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly
struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    uint32_t usedHeap;             // heap usage mask that shows regions used from mpu blocks 31-4
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    uint32_t stackSize;            // stack size
    void *usedSemaphore[MAX_SEMAPHORES];            // if it has used a resource
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

// this is the access to the heap
extern uint32_t heapMem[];

// CPU time tables
uint32_t timeTable[2][MAX_TASKS+1];
uint8_t liveTableSet = 0;
bool printingTime = false;

typedef struct _psData
{
    void* pid;
    char name[16];
    uint16_t timePCint;
    uint16_t timePCdec;
    uint8_t state;
}psData;
typedef struct _ipcsData
{
    char name[16];
    uint16_t count;
    uint16_t queueSize;
    char waitingName[MAX_TASKS][16];
}ipcsData;

// uses Uart0 to print 4 bytes in hex MSB-LSB
void print4ByteHex(uint32_t bytes)
{
    char hexStr[9];
    u32ToStrHex(hexStr, bytes);
    putsUart0("0x");
    putsUart0(hexStr);
}
// uses Uart0 to print a byte in hex
void printByteHex(uint8_t byte)
{
    char hexStr[9];
    intToStrHex(hexStr, byte);
    putsUart0("0x");
    putsUart0(hexStr);
}
void printU32(uint32_t number)
{
    char numStr[20];
    u32ToStr(numStr, number);
    putsUart0(numStr);
}
// returns unused heap blocks
uint32_t availableHeap()
{
    uint32_t i, heapMask;
    heapMask = 0x0000000F;

    // go through all tasks and mark used heap blocks in mask
    for(i=0; i<MAX_TASKS; i++)
    {
        heapMask |= tcb[i].usedHeap;
    }
    // return unused heap blocks
    return ~heapMask;
}

// return top memory location of available block(s) needed from the SRAM blocks 31-4 (heap)
void* getOpenHeapAddr(uint32_t *usedHeap, uint32_t freeMask, uint32_t size)
{
    uint8_t blocksNeeded = size/1024;
    uint8_t consecutiveBlocks = 0;
    uint32_t region = 0x10;
    uint32_t usageMask = 0x0;
    uint8_t i = 4;

    if(size % 1024)
        blocksNeeded += 1;

    while(region != 0x00000000)
    {
        if(freeMask & region)
        {
            consecutiveBlocks++;
            usageMask |= region;
            if(consecutiveBlocks == blocksNeeded)
            {
                *usedHeap = usageMask;
                return getHeapRegionAddr(i);
            }
        }
        else
            consecutiveBlocks = 0;
        i++;
        region = region << 1;
    }
    return 0;
}
void enableSystick()
{
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
}
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records & time tables
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
        timeTable[0][i] = 0;
        timeTable[1][i] = 0;
    }
    timeTable[0][MAX_TASKS] = timeTable[1][MAX_TASKS] = 0;
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    uint8_t lastTask = task;
    uint8_t consideredTask = 0xFF;

    if(lastTask == 0xFF)
    {
        lastTask = taskCount-1;
    }
    ok = false;
    task += 1;
    task %= taskCount;
    while (!ok)
    {
        if(task == lastTask && consideredTask != 0xFF)
        {
            ok = true;
        }
        if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
        {
            if(consideredTask == 0xFF || tcb[task].currentPriority < tcb[consideredTask].currentPriority)
            {
                consideredTask = task;
            }
        }
        task += 1;
        task %= taskCount;
    }
    task = consideredTask;
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].spInit = tcb[i].sp = getOpenHeapAddr(&tcb[i].usedHeap, availableHeap(), stackBytes) - 12;
            tcb[i].priority = tcb[i].currentPriority = priority;
            tcb[i].stackSize = stackBytes;
            c_strcpy(tcb[i].name, name);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}
// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
//    uint32_t pidNum;
//    char pidStr[11];
//    pidNum = (uint32_t) tcb[0].pid;
//    u32ToStrHex(pidStr, pidNum);
//    putsUart0(pidStr);
//    putsUart0("\n\r");
    uint32_t sp;

    taskCurrent = rtosScheduler();
    sp = tcb[taskCurrent].spInit;
    setPSP(sp);
    tcb[taskCurrent].state = STATE_READY;
    setControl(0x2);                                // setting ASP for PSP
    _fn fn1 = tcb[taskCurrent].pid;
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn on general timer
    enableSystick();
    enableMemBlocks(tcb[taskCurrent].usedHeap);
    setControl(0x3);

    fn1();
}
// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm("     SVC     #20");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    __asm("     SVC     #21");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm("     SVC     #9");
}

int8_t createSemaphore(uint8_t count, char* semaphoreName)
{
    int8_t index = -1;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        c_strcpy(semaphores[semaphoreCount].name, semaphoreName);
        semaphores[semaphoreCount].count = count;
        semaphores[semaphoreCount].lastTask = 0xFF;
        index = semaphoreCount;
        semaphoreCount++;
    }
    return index;
}


// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm("     SVC      #1");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm("     SVC      #11");
}


// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm("     SVC     #33");
}
// REQUIRED: modify this function to wait for a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm("     SVC     #22");
}
void ps(psData* pD, uint8_t* numtasks)
{
    __asm("     SVC     #2");
}
void ipcs(ipcsData* iD, uint8_t* numSemaphores)
{
    __asm("     SVC     #3");
}
void kill(void* pid)
{
    __asm("     SVC     #4");
}
void run(char* name)
{
    __asm("     SVC     #5");
}
void pidof(char* name , void* pid)
{
    __asm("     SVC     #6");
}
void setPreempt(uint32_t preempt)
{
    __asm("     SVC     #7");
}
void setPi(uint32_t pi)
{
    __asm("     SVC     #8");
}
void osHeap(uint8_t heapSpace[])
{
    __asm("     SVC     #50");
}
// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void SystickISR(void)
{
//    char pidNum[2];
//    intToStr(pidNum, taskCurrent);
//    putsUart0("SysTick in process ");
//    putsUart0(pidNum);
    int i;
    static ticks=0;
    ticks++;
    if(ticks > 20 && !printingTime)
    {
        liveTableSet = 1 - liveTableSet;
        for(i=0; i<MAX_TASKS+1; i++)
        {
            timeTable[liveTableSet][i] = 0;
        }
        ticks=0;
    }
    //if preemption is enabled, allow task switch
    if(preemptEnabled)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
    for(i=0; i<MAX_TASKS; i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks -= 1;
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void PendsvISR(void)
{
    uint32_t time;

    //Save other registers not saved automatically
    pushR4toR11toPSP();
    tcb[taskCurrent].sp = getPSP();

    //Save CPU time
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    __delay_cycles(6);

    TIMER0_TAV_R /= 100;
    time = TIMER0_TAV_R & 0xFFFFFFFF;
    timeTable[liveTableSet][taskCurrent] += time;
    timeTable[liveTableSet][MAX_TASKS] += time;

    taskCurrent = rtosScheduler();

    //switch to new task's stack
    setPSP((uint32_t) tcb[taskCurrent].sp);

    // switch memory block permissions
//    enableMemBlocks(0xFFFFFFF0);
    enableMemBlocks(tcb[taskCurrent].usedHeap);
    //if task has already been run before, switch normally
    if(tcb[taskCurrent].state == STATE_READY)
    {
        popR4toR11fromPSP();
    }
    //if task has not yet been run, fill registers with appropriate data to switch to the new task
    else if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        tcb[taskCurrent].state = STATE_READY;
        __asm("     MRS r4,     PSP");

        // pushing the xPSR with thumb state selected
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r5,     #0");
        __asm("     MOVT r5,     #0x0100");
        __asm("     STR r5,     [r4]");

        __asm("     MSR PSP,    r4");
        //pushing PC with function pointer of new function
        pushPC((uint32_t)tcb[taskCurrent].pid);
        __asm("     MRS r4,     PSP");

        // pushing LR with 0xFFFF.FFFD for returning to thread mode w/o FP stack
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV LR,     #0");
        __asm("     SUB LR,     #3");
        __asm("     STR LR,     [r4]");
        // pushing R12 with 12
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r12,    #12");
        __asm("     STR r12,    [r4]");
        // pushing R3 with 3
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r3,     #3");
        __asm("     STR r3,     [r4]");
        // pushing R2 with 2
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r2,     #2");
        __asm("     STR r2,     [r4]");
        // pushing R1 with 1
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r1,     #1");
        __asm("     STR r1,     [r4]");
        // pushing R0 with 0
        __asm("     SUB r4,     r4, #4");
        __asm("     MOV r0,     #0");
        __asm("     STR r0,     [r4]");
        __asm("     MSR PSP,    r4");
    }
    // check DERR & IERR bits in Fault Status register
    if(getMemFaultFlags() & 0x3)
    {
        clearMemFault();
        putsUart0("Called by MPU\n\r");
    }
    // restart timer for CPU time
    TIMER0_TAV_R = 0;
    TIMER0_CTL_R |= TIMER_CTL_TAEN;
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void SVCallISR(void)
{

    // getting SVC number code from instruction format (last 8 bits)
    uint8_t svcCode;
    svcCode = getSVC() & 0xFF;

    uint32_t waitTime;
    uint8_t currentSemaphore;
    uint32_t *dataPointer;
    int i,j;

    switch(svcCode)
    {
    case 1:     // Yield: allow other available tasks to run
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    case 11:    // sleep: set a minimum amount of time that needs to elapse for a task to be ready again
        waitTime = getR0();
        tcb[taskCurrent].ticks = waitTime;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    case 22:    // wait: take resource from semaphore when available or join the queue
        currentSemaphore = getR0();
        if(semaphores[currentSemaphore].count > 0)
        {
            semaphores[currentSemaphore].count -= 1;
            semaphores[currentSemaphore].lastTask = taskCurrent;
            tcb[taskCurrent].usedSemaphore[currentSemaphore] = &semaphores[currentSemaphore];
        }
        else
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            tcb[taskCurrent].semaphore = &semaphores[currentSemaphore];

            //if there is a valid last task
            if(piEnabled && (semaphores[currentSemaphore].lastTask <= taskCount))
            {
//                if(tcb[semaphores[currentSemaphore].lastTask].state == STATE_READY)
//                {
                    //if priority inheritance is enabled, increase priority of last task to unblock higher priority task waiting for semaphore
                    if(tcb[semaphores[currentSemaphore].lastTask].currentPriority < tcb[taskCurrent].currentPriority)
                    {
                        tcb[semaphores[currentSemaphore].lastTask].currentPriority = tcb[taskCurrent].currentPriority;
                    }
//                }
            }
            if(semaphores[currentSemaphore].queueSize < MAX_QUEUE_SIZE)
            {
                //adding task to semaphore queue
                semaphores[currentSemaphore].processQueue[ semaphores[currentSemaphore].newIndex ] = taskCurrent;
                //setting new entry index within the range of the max queue size
                semaphores[currentSemaphore].newIndex += 1;
                semaphores[currentSemaphore].newIndex %= MAX_QUEUE_SIZE;
                //increasing current number of tasks in queue
                semaphores[currentSemaphore].queueSize += 1;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }
            else
            {
                putsUart0("Semaphore ");
                print4ByteHex(currentSemaphore);
                putsUart0(": Queue is currently filled. Discarding Task's Queue Request.\n\r");
            }
        }
        break;

    case 33:    // post: basically releasing one resource from semaphore
        // getting the semaphore needed from SVC instruction
        currentSemaphore = getR0();
        semaphores[currentSemaphore].count += 1;
        tcb[taskCurrent].usedSemaphore[currentSemaphore] = 0;

        // restore priority of the current task to original priority in case of priority inheritance
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;

        // checking if there are any tasks waiting for a resource
        if(semaphores[currentSemaphore].queueSize > 0)
        {
            uint32_t task;
            // get waiting task from queue
            task = semaphores[currentSemaphore].processQueue[ semaphores[currentSemaphore].oldestIndex ];

            // update oldest task in the queue
            semaphores[currentSemaphore].oldestIndex += 1;
            semaphores[currentSemaphore].oldestIndex %= MAX_QUEUE_SIZE;

            // allow task waiting to be ready for the resource now available
            tcb[task].state = STATE_READY;
            tcb[task].semaphore = 0;
            semaphores[currentSemaphore].count -= 1;
            semaphores[currentSemaphore].queueSize -= 1;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;

    // ### Shell Cases ###
    case 100:   //reboot
        NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
        break;

    case 2: //ps command
        // TODO Check if struct is still in the shell stack
        if(1)
        {
            uint32_t tempTime;
            // struct for ps data
            psData *pD = getR0();
            //task index, returned with number of tasks running
            uint8_t* numTasks = getR1();
            if(*numTasks == 0)
                printingTime = true;
            tempTime = (timeTable[1-liveTableSet][*numTasks]*10000);
            tempTime /= (timeTable[1-liveTableSet][MAX_TASKS]);

            c_strcpy(pD->name, tcb[*numTasks].name);
            pD->pid = tcb[*numTasks].pid;
            pD->timePCint = tempTime / 100;
            pD->timePCdec = tempTime % 100;
            pD->state = tcb[*numTasks].state;
            if(*numTasks == taskCount-1)
                printingTime = false;
            *numTasks = taskCount;
        }
        break;

    case 3: //ipcs
        //TODO: Check if the struct is in the shell stack
        if(1)
        {
            ipcsData* semaphoreData = getR0();
            uint8_t* index = getR1();
            uint16_t waitingTaskId;
            uint16_t queueIndex;

            c_strcpy(semaphoreData->name, semaphores[*index].name);
            semaphoreData->count = semaphores[*index].count;
            semaphoreData->queueSize = semaphores[*index].queueSize;

            queueIndex = semaphores[*index].oldestIndex;
            for(i=0; i<semaphoreData->queueSize; i++)
            {
                waitingTaskId = semaphores[*index].processQueue[queueIndex];
                c_strcpy(semaphoreData->waitingName[i], tcb[waitingTaskId].name);
                queueIndex++;
                queueIndex %= MAX_QUEUE_SIZE;
            }
            *index = semaphoreCount;
        }
        break;
    case 4: //kill
        for(i=0; i<taskCount; i++)
        {
            // check if the pid to be killed is any of the existing tasks' pid
            if(tcb[i].pid == getR0())
            {
                if(i == 0)
                    putsUart0("Error kill: Cannot kill idle process.\n\r");
                else
                {
                    if(tcb[i].state != STATE_INVALID)
                    {
//                        tcb[i].state = STATE_INVALID;
                        goto destroyStuff;
                        killBack:
                        putsUart0("Process ");
                        printU32(getR0());
                        putsUart0(": has been killed\n\r");
                        return;
                    }
                }
            }
        }
        putsUart0("Error kill: Non-existent program\n\r");
        break;

    case 5: // run
        for(i=0; i<taskCount; i++)
        {
            if(c_strcmp(tcb[i].name, (char*) getR0()) == 0)
            {
                if(tcb[i].state == STATE_INVALID)
                {
                    goto restartStuff;
//                    tcb[i].sp = tcb[i].spInit;
//                    tcb[i].state = STATE_UNRUN;
                    runBack:
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                }
                else
                {
                    putsUart0("Task already running.\n\r");
                }
                return;
            }
        }
        putsUart0("Error run: Non-existent program\n\r");
        break;

    case 6: //pidof

        dataPointer = getR1();
        for(i=0; i<taskCount; i++)
        {
            if(c_strcmp(tcb[i].name, (char*) getR0()) == 0)
            {
                *dataPointer = tcb[i].pid;
                break;
            }
        }
        break;
    case 7: //preemption
        preemptEnabled = getR0();
        break;
    case 8: //pi
        piEnabled = getR0();
        break;
    case 9: // set priority
        for(i=0; i<taskCount; i++)
        {
            // find the task to be promoted in priority
            if(tcb[i].pid == getR0())
            {
                // set task priority to value passed
                tcb[i].currentPriority = getR1();
                break;
            }
        }
        break;
    case 20: // restart thread
        for(i=0; i<taskCount; i++)
        {
            // find the task to be restarted
            if(tcb[i].pid == getR0())
            {
                restartStuff:
                tcb[i].spInit = tcb[i].sp = getOpenHeapAddr(&tcb[i].usedHeap, availableHeap(), tcb[i].stackSize);
                tcb[i].state = STATE_UNRUN;
                break;
            }
        }
        if(svcCode == 5)
            goto runBack;
        break;
    case 21: // destroy thread
        for(i=0; i<taskCount; i++)
        {
            // find the task to be destroyed
            if(tcb[i].pid == getR0())
            {
                semaphore *tempSemaphore;
                destroyStuff:
                //clear from any semaphore blocks
                if(tcb[i].semaphore != 0)
                {
                    uint8_t index  = 0;
                    uint8_t index2 = 0;

                    tempSemaphore = tcb[i].semaphore;
                    //find task in semaphore waiting queue in the tcb
                    index = tempSemaphore->oldestIndex;
                    for(j=0; j<tempSemaphore->queueSize; j++)
                    {
                        if(tempSemaphore->processQueue[index] == i)
                            break;
                        index += 1;
                        index %= MAX_QUEUE_SIZE;
                    }
                    //if found
                    if(j<tempSemaphore->queueSize)
                    {
                        index2 = index;
                        //shift everything up after removing task from waiting queue
                        while(index != tempSemaphore->oldestIndex)
                        {
                            //loop around to end of array if reached the first value
                            if(index2 == 0)
                                index2 = MAX_QUEUE_SIZE;
                            else
                                index2--;
                            tempSemaphore->processQueue[index] = tempSemaphore->processQueue[index2];
                            index = index2;
                        }
                        //update oldestIndex since everything was shifted
                        tempSemaphore->oldestIndex += 1;
                        tempSemaphore->oldestIndex %= MAX_QUEUE_SIZE;
                        tempSemaphore->queueSize--;
                    }
                }
                // post semaphores used by task if it was killed before it could post
                for(j=0; j<semaphoreCount; j++)
                {
                    if(tempSemaphore = tcb[i].usedSemaphore[j])
                    {
                        tempSemaphore->count++;
                        //free a function in waiting queue
                        // checking if there are any tasks waiting for a resource
                        if(tempSemaphore->queueSize > 0)
                        {
                            uint32_t task;
                            // get waiting task from queue
                            task = tempSemaphore->processQueue[ tempSemaphore->oldestIndex ];

                            // update oldest task in the queue
                            tempSemaphore->oldestIndex += 1;
                            tempSemaphore->oldestIndex %= MAX_QUEUE_SIZE;

                            // allow task waiting to be ready for the resource now available
                            tcb[task].state = STATE_READY;
                            tcb[task].semaphore = 0;
                            tempSemaphore->count -= 1;
                            tempSemaphore->queueSize -= 1;
                        }
                    }
                }
                // clear heap usage and stack pointer
                tcb[i].sp = tcb[i].spInit = tcb[i].usedHeap = 0;
                tcb[i].state = STATE_INVALID;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                if(svcCode == 4)
                    goto killBack;
                break;
            }
        }
        break;

    case 50:
        if(1)
        {
            uint8_t *heap;
            uint32_t bitmask;
            heap = (uint8_t*)getR0();

            for(j=0; j<28; j++)
                heap[j] = 0xFF;
            for(i=0; i<taskCount; i++)
            {
                // begin search at block 4 to block 32 of the heap
                bitmask = 0x10;
                for(j=4; j<32; j++)
                {
                    // mark used heap blocks with task indexes
                    if(tcb[i].usedHeap & bitmask)
                        heap[j-4] = i;
                    bitmask = bitmask << 1;
                }
            }
        }
        break;
    default: putsUart0("SVC Error: No valid service\n\r");
    }
}

// REQUIRED: code this function
void MpuISR()
{
    char pidNum[2];

    clearMemPending();
    intToStr(pidNum, taskCurrent);
    putsUart0("Mpu fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

    // Displaying Stack pointer positions and Error information
    putsUart0("PSP:\t");
    print4ByteHex(getPSP());
    putsUart0("\n\rMSP:\t");
    print4ByteHex(getMSP());
    putsUart0("\n\rMFF:\t");
    printByteHex(getMemFaultFlags());
    putsUart0("\n\r");

    //Register Dump
    int i;
    uint32_t* psp = (uint32_t*) getPSP();
    for(i=0; i<8; i++)
    {
        switch(i)
        {
        case 0: putsUart0("R0:\t");  break;
        case 1: putsUart0("R1:\t");  break;
        case 2: putsUart0("R2:\t");  break;
        case 3: putsUart0("R3:\t");  break;
        case 4: putsUart0("R12:\t"); break;
        case 5: putsUart0("LR:\t");  break;
        case 6: putsUart0("PC:\t");  break;
        case 7: putsUart0("XPSR:\t");break;
        }
        print4ByteHex(*(psp + i));
        putsUart0("\n\r");
    }
    //displaying instruction causing violation
    putsUart0("Intruction ");
    print4ByteHex(*(psp+6));
    putsUart0(" accessed ");
    print4ByteHex(getMemAccessAddr());
    putsUart0("\n\r");

    tcb[taskCurrent].state = STATE_INVALID;
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: code this function
void FaultISR()
{
    char pidNum[2];
    intToStr(pidNum, taskCurrent);
    putsUart0("hard fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

    putsUart0("PSP:\t");
    print4ByteHex(getPSP());
    putsUart0("\n\rMSP:\t");
    print4ByteHex(getMSP());
    putsUart0("\n\rHFF:\t");
    print4ByteHex(getHFaultFlags());
    putsUart0("\n\r");

    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_BUSA;
}

// REQUIRED: code this function
void BusISR(void)
{
    char pidNum[2];
    intToStr(pidNum, taskCurrent);
    putsUart0("Bus fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

}

// REQUIRED: code this function
void UsageISR(void)
{
    char pidNum[2];
    intToStr(pidNum, taskCurrent);
    putsUart0("Usage fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

}
void PrivilegeISR(void)
{
    setControl(0x2);
    GPIO_PORTE_ICR_R |= 0xFF;
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    _delay_cycles(3);

    SYSCTL_RCGCTIMER_R = SYSCTL_RCGCTIMER_R0;
    _delay_cycles(6);

    enablePort(PORTA);
    enablePort(PORTE);
    enablePort(PORTF);

    selectPinPushPullOutput(RED_LED);       // RED LED
    selectPinPushPullOutput(ORANGE_LED);    // ORANGE LED
    selectPinPushPullOutput(YELLOW_LED);    // YELLOW LED
    selectPinPushPullOutput(GREEN_LED);     // GREEN LED
    selectPinPushPullOutput(BLUE_LED);      // BLUE LED(ON BOARD)
    selectPinPushPullOutput(PRIVILEGE);     // PRIVILEGE

    enablePinInterrupt(PRIVILEGE);
    selectPinInterruptFallingEdge(PRIVILEGE);
    NVIC_EN0_R |= (1 << 4);

    enablePinPullup(BUTTON_0);
    enablePinPullup(BUTTON_1);
    enablePinPullup(BUTTON_2);
    enablePinPullup(BUTTON_3);
    enablePinPullup(BUTTON_4);
    enablePinPullup(BUTTON_5);

    selectPinDigitalInput(BUTTON_0);
    selectPinDigitalInput(BUTTON_1);
    selectPinDigitalInput(BUTTON_2);
    selectPinDigitalInput(BUTTON_3);
    selectPinDigitalInput(BUTTON_4);
    selectPinDigitalInput(BUTTON_5);

    NVIC_ST_RELOAD_R = 40000-1;
    NVIC_ST_CURRENT_R = 40000-1;

    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                                        // turn off timer
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;                                  // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;             // count up Periodic timer
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t total;
    total = 0;
    total += (uint8_t) getPinValue(BUTTON_5)^1;
    total = total << 1;
    total += (uint8_t) getPinValue(BUTTON_4)^1;
    total = total << 1;
    total += (uint8_t) getPinValue(BUTTON_3)^1;
    total = total << 1;
    total += (uint8_t) getPinValue(BUTTON_2)^1;
    total = total << 1;
    total += (uint8_t) getPinValue(BUTTON_1)^1;
    total = total << 1;
    total += (uint8_t) getPinValue(BUTTON_0)^1;

    return total;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
void setPrivilegeMode(bool on)
{
    if(on)
    {
        setPinValue(PRIVILEGE, 1);
        setPinValue(PRIVILEGE, 0);
    }
    else
    {
        setControl(0x3);
    }
}
void testISRs(uint8_t type)
{
    /*
     * 0: Usage fault
     * 1: bus fault
     * 2: hard fault
     * 3: mpu fault
     */

    uint32_t* p;
    switch(type)
    {
    case 0:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGEP;
            setPrivilegeMode(false);
        }
        else
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGEP;
        break;
    case 1:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUSP;
            setPrivilegeMode(false);
        }
        else
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUSP;
        break;
    case 2:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEM);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEMP;
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM;
            setPrivilegeMode(false);
        }
        else
        {
            NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEM);
            setPrivilegeMode(false);
            p = (uint32_t*) 0x20000FFC;
            *p = 0;
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= (NVIC_SYS_HND_CTRL_MEM);
        }
        break;
    case 3:
        //not protected
        p = (uint32_t*) 0x20007FFC;
        *p = 0;
        //protected
        p = (uint32_t*) 0x20000FFC;
        *p = 0;
        break;
    }
}
void testHw()
{

    while(getPinValue(BUTTON_0));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_1));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);

    while(getPinValue(BUTTON_2));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_3));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);

    while(getPinValue(BUTTON_4));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_5));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);
}
void isCommand(char** fields, uint8_t num_fields)
{
    char* command = fields[0];
    if(c_strcmp(command, "reboot") == 0)
    {
        if(num_fields > 1)
            putsUart0("reboot: invalid argument\n\r");
        else
        {
            __asm("     SVC     #100");
//            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
        }
    }
    else if(c_strcmp(command, "ps") == 0)
    {
        if(num_fields > 1)
            putsUart0("ps: invalid argument\n\r");
        else
        {
            uint8_t numTasks, i;
            psData taskData;
            i=0;
            putsUart0("|#|\t| PID |\t| NAME |\t| CPU % |\n\r");
            do
            {
                numTasks = i;
                ps(&taskData, &numTasks);
                if(taskData.state != STATE_INVALID)
                {
                    printU32(i);
                    putsUart0(":\t");
                    printU32(taskData.pid);
                    putsUart0("\t");
                    putsUart0(taskData.name);
                    putsUart0("\t");
                    if(c_strlen(taskData.name) < 8)
                        putsUart0("\t");
                    printU32(taskData.timePCint);
                    putsUart0(".");
                    if(taskData.timePCdec/10 == 0)
                        putsUart0("0");
                    printU32(taskData.timePCdec);
                    putsUart0("\n\r");
                }
                i++;
            }while(i<numTasks);
        }
    }
    else if(c_strcmp(command, "ipcs") == 0)
    {
        if(num_fields > 1)
            putsUart0("ipcs: invalid argument\n\r");
        else
        {
            uint8_t i,j, numSemaphores;
            ipcsData semaphoreData;

            putsUart0("| NAME |\t| COUNT |\t| WAITING QUEUE |\n\r");
            i = j = 0;
            do
            {
                numSemaphores = i;
                ipcs(&semaphoreData, &numSemaphores);
                putsUart0(semaphoreData.name);
                putsUart0("\t");
                if(c_strlen(semaphoreData.name) < 8)
                {
                    putsUart0("\t");
                }
                printU32(semaphoreData.count);
                putsUart0("\t\t");
                if(semaphoreData.queueSize > 0)
                {
                    for(j=0; j<semaphoreData.queueSize; j++)
                    {
                        putsUart0(semaphoreData.waitingName[j]);
                        putsUart0("\n\r\t\t\t\t");
                    }
                    putsUart0("\r");
                }
                else
                    putsUart0("\n\r");
                i++;
            }while(i<numSemaphores);
//            putsUart0("IPCS called\n\r");
        }
    }
    else if(c_strcmp(command, "kill") == 0)
    {
        if(num_fields != 2 || (num_fields == 2 && !isNumber(fields[1])) )
            putsUart0("kill: invalid argument\n\r");
        else
        {
            if(isNumber(fields[1]))
            {
                kill(strToInt(fields[1]));
            }
        }
    }
    else if(c_strcmp(command, "pi") == 0)
    {
        if(num_fields != 2)
            putsUart0("pi: invalid argument\n\r");
        else
        {
            if(c_strcmp(fields[1], "on") == 0)
            {
                setPi(true);
            }
            else if(c_strcmp(fields[1], "off") == 0)
            {
                setPi(false);
            }
            else
            {
                putsUart0("pi: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "preempt") == 0)
    {
        if(num_fields != 2)
            putsUart0("preempt: invalid argument");
        else
        {
            if(c_strcmp(fields[1], "on") == 0)
            {
                setPreempt(true);
            }
            else if(c_strcmp(fields[1], "off") == 0)
            {
                setPreempt(false);
            }
            else
            {
                putsUart0("preempt: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "sched") == 0)
    {
        if(num_fields != 2)
            putsUart0("sched: invalid argument\n\r");
        else
        {
            if(c_strcmp(fields[1], "prio") == 0)
            {
                putsUart0("sched prio\n\r");
            }
            else if(c_strcmp(fields[1], "rr") == 0)
            {
                putsUart0("sched rr\n\r");
            }
            else
            {
                putsUart0("sched: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "pidof") == 0)
    {
        if(num_fields != 2)
            putsUart0("pidof: invalid argument\n\r");
        else
        {
            uint32_t pid;
            pid = 0;
            pidof(fields[1], &pid);
            if(pid)
            {
                putsUart0(fields[1]);
                putsUart0(" : ");
                printU32(pid);
                putsUart0("\n\r");
            }
            else
            {
                putsUart0("Error: Non-existent program.\n\r");
            }

        }
    }
    else if(c_strcmp(command, "run") == 0)
    {
        if(num_fields != 2)
            putsUart0("run: invalid argument\n\r");
        else
        {
            run(fields[1]);
        }
    }
    else if(c_strcmp(command, "heap") == 0)
    {
        uint8_t usedHeap[28];
        int i;
        osHeap(usedHeap);
        putsUart0("| Heap Space Usage |\n\r");
        for(i=27; i>=0; i--)
        {
           printU32(i+4);
           putsUart0("\t");
           putsUart0("[ ");
           printU32(usedHeap[i]);
           putsUart0(" ]\n\r");
        }
    }
//    else if(c_strcmp(command, "test") == 0)
//    {
//        if(num_fields == 2)
//        {
//            if(isNumber(fields[1]))
//            {
//                testISRs((uint8_t) strToInt(fields[1]));
//            }
//            else if(c_strcmp(fields[1], "hw") == 0)
//            {
//                testHw();
//            }
//        }
//        else
//        {
//            putsUart0("test: Invalid argument\n\r");
//        }
//    }
    else
    {
        putsUart0("Invalid Command\n\r");
        return;
    }
}
// returns the number of fields
uint32_t parseFields(char* buffer, char** fields)
{
    char* delim = " ,";
    uint8_t bufferSize;
    bufferSize = c_strlen(buffer);
    uint8_t c_index = 0;
    uint8_t f_index = 0;

    if(bufferSize == 0)
        return 0;

    //check first index
    if(isDelim(buffer[c_index], delim))
    {
        buffer[c_index] = '\0';
    }
    else
    {
        fields[f_index] = buffer;
        f_index++;
    }
    c_index++;
    //everything after, if prefixed with delim, it is a new token. Max fields 5
    while(c_index < bufferSize && f_index < 5)
    {
        if(isDelim(buffer[c_index], delim))
        {
            buffer[c_index] = '\0';
        }
        else
        {
            if(buffer[c_index-1] == '\0')
            {
                fields[f_index] = &buffer[c_index];
                f_index++;
            }
        }
        c_index++;
    }
    return f_index;
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)               // wait if uart0 rx fifo empty
        sleep(50);
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
bool getsUart0(char* saveBuffer, uint8_t* index)
{
    char c;
    c = getcUart0();
    yield();
    if(c == 13 || *index >= 80)
    {
        saveBuffer[*index] = '\0';
        *index = 0;
        return true;
    }
    putcUart0(c);
    if(c == '\b')
    {
        putcUart0(' ');
        putcUart0('\b');
        if(*index > 0)
            *index -= 1;
    }
    else
    {
        saveBuffer[*index] = c;
        *index += 1;
    }
    return false;
}
void testPbs()
{
    int num;
    char numStr[4];

    num = (int) readPbs();
    intToStr(numStr, num);
    putsUart0(numStr);
    putsUart0("\n\r");
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

    while(true)
    {
//        putsUart0("\n\r");
//        print4ByteHex(getPSP());
//        putsUart0("\n\r");

        setPinValue(ORANGE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(ORANGE_LED, 0);
//        putsUart0("1\n\r");
        yield();
    }
}
void idle2()
{
    while(true)
    {
//        putsUart0("\n\r");
//        print4ByteHex(getPSP());
//        putsUart0("\n\r");

//        setPinValue(YELLOW_LED, 1);
//        waitMicrosecond(1000);
//        setPinValue(YELLOW_LED, 0);
//        sleep(1000);
        yield();
    }
}
void idle3()
{
    while(true)
    {
        sleep(1);
    }
}
void idle4()
{
    putsUart0("4\n\r");
    yield();
}
void idle5()
{
    putsUart0("5\n\r");
    yield();
}
void idle6()
{
    putsUart0("6\n\r");
    yield();
}

void flash4Hz()
{
    while(true)
    {
        setPinValue(GREEN_LED, getPinValue(GREEN_LED)^1);
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(FLASHREQ);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void oneshot2()
{
    while(true)
    {
        wait(FLASHREQ);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(RESOURCE);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        setPinValue(RED_LED, getPinValue(RED_LED)^1);
        post(RESOURCE);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(KEYRELEASED);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(KEYPRESSED);
        if ((buttons & 1) != 0)
        {
            setPinValue(YELLOW_LED, getPinValue(YELLOW_LED)^1);
            setPinValue(RED_LED, 1);
        }
        if ((buttons & 2) != 0)
        {
            post(FLASHREQ);
            setPinValue(RED_LED, 0);
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(KEYPRESSED);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(KEYRELEASED);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(RESOURCE);
        setPinValue(BLUE_LED, 1);
        sleep(1000);
        setPinValue(BLUE_LED, 0);
        post(RESOURCE);
    }
}
// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    putsUart0("\n\rCSE 4354: RTOS\n\n\n\n\n\n\n\n\n\n\n\n\n\r");
    char commandLine[80];               // had to make constant cause error due to variable?
    char* fields[5];                    // ^^^^
    uint8_t numFields = 0;
    uint8_t index = 0;

    //run endless loop for commands
    while(1)
    {
        while(!getsUart0(commandLine, &index));
        putsUart0("\n\r");
        numFields = parseFields(commandLine, fields);
        if(numFields > 0)
        {
            isCommand(fields, numFields);
        }
        commandLine[0] = '\0';
        numFields = 0;
    }
}
bool initBaseThreads()
{
    bool ok;
    // Add required idle process at lowest priority
    ok = createThread(idle, "Idle", 15, 1024);
//    //testing priority stuff
//    ok = createThread(idle2, "Idle2", 8, 1024);
//    ok = createThread(idle3, "Idle3", 4, 1024);
//    ok = createThread(idle4, "Idle4", 0, 1024);
//    ok = createThread(idle5, "Idle5", 8, 1024);
//    ok = createThread(idle6, "Idle6", 4, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
//    ok &= createThread(oneshot2, "OneShot2", 8, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(errant, "Errant", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);

    return ok;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);


    initMpu();                                      // setting up memory protection regions
    enableSysFaults();                              // enabling system faults

    // Power-up flash
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(250000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1, "keyPressed");
    keyReleased = createSemaphore(0, "keyReleased");
    flashReq = createSemaphore(5, "flashReq");
    resource = createSemaphore(1, "resource");

    // Start up RTOS
    if (initBaseThreads())
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);
    while(1);

    return 0;
}
