/*
    All rights reserved
*/
#include "types.h"
#include "multi_timer.h"
#include "osal.h"
#include "osal_memory.h"

//timer handle list head.
struct multiTimer* head_handle = NULL;

//Timer ticks
static uint32 _timer_ticks = 0;

/**
    @brief  Initializes the timer struct handle.
    @param  handle: the timer handle strcut.
    @param  timeout_cb: timeout callback.
    @param  repeat: repeat interval time.
    @retval None
*/
void multitimer_init(struct multiTimer* handle, void(*timeout_cb)(uint16 idx), uint32 timeout, uint32 repeat,uint32 id)
{
    osal_memset(handle, sizeof(struct multiTimer), 0);
    handle->timeout_cb = timeout_cb;
    handle->timeout = _timer_ticks + timeout;
    handle->repeat = repeat;
    handle->id = id;
}

/**
    @brief  Start the timer work, add the handle into work list.
    @param  btn: target handle strcut.
    @retval 0: succeed. -1: already exist.
*/
int multitimer_start(struct multiTimer* handle)
{
    struct multiTimer* target = head_handle;

    while(target)
    {
        if(target == handle) return -1; //already exist.

        target = target->next;
    }

    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

/**
    @brief  Stop the timer work, remove the handle off work list.
    @param  handle: target handle strcut.
    @retval None
*/
void multitimer_stop(struct multiTimer* handle)
{
    struct multiTimer** curr;

    for(curr = &head_handle; *curr; )
    {
        struct multiTimer* entry = *curr;

        if (entry == handle)
        {
            *curr = entry->next;
            osal_mem_free(entry);
        }
        else
            curr = &entry->next;
    }
}

/**
    @brief  main loop.
    @param  None.
    @retval None
*/
void multitimer_loop()
{
    struct multiTimer* target;

    for(target=head_handle; target; target=target->next)
    {
        if(_timer_ticks >= target->timeout)
        {
            target->timeout_cb(target->id);

            if(target->repeat == 0)
            {
                multitimer_stop(target);
            }
            else
            {
                target->timeout = _timer_ticks + target->repeat;
            }
        }
    }
}

/**
    @brief  background ticks, timer repeat invoking interval 1ms.
    @param  None.
    @retval None.
*/
void multitimer_ticks( uint32 tick )
{
    _timer_ticks += tick ;
}

