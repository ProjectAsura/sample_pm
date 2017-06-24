//-------------------------------------------------------------------------------------------------
// File : r3d_stack_allocator.h
// Desc : Stack Allocator.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------
#pragma once

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include <malloc.h>


///////////////////////////////////////////////////////////////////////////////////////////////////
// stack_allocator class
///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
struct stack_allocator
{
    using value_type = T;

    stack_allocator()
    { /* DO_NOTHING */ }

    template<typename U>
    stack_allocator(const stack_allocator<U>&)
    { /* DO_NOTHING */ }

    T* allocate(size_t count)
    { return reinterpret_cast<T*>(alloca(sizeof(T) *count)); }

    void deallocate(T*, size_t)
    { /* DO_NOTHING */ }
};

template<typename T, typename U>
inline bool operator == (const stack_allocator<T>&, const stack_allocator<U>&)
{ return true; }

template<typename T, typename U>
inline bool operator != (const stack_allocator<T>&, const stack_allocator<U>&)
{ return false; }
