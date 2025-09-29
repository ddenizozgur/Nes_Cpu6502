#if !defined(_DARRAY_H)
#define _DARRAY_H

#include <string.h>
#include <stdlib.h>

typedef enum {
    DARRAY_SUCCESS,
    DARRAY_ON_EMPTY,
    DARRAY_ALLOC_FAILURE,
} DARRAY_;

#define DARRAY_DEFINE(struct_tag, func_tag, data_type)                          \
    typedef struct {                                                            \
        size_t size, capacity;                                                  \
        data_type *data;                                                        \
    } struct_tag;                                                               \
                                                                                \
    struct_tag func_tag##_init() {                                              \
        return (struct_tag) { 0 };                                              \
    }                                                                           \
                                                                                \
    DARRAY_ func_tag##_free(struct_tag *darray) {                               \
        if (darray->data == NULL)                                               \
            return DARRAY_ON_EMPTY;                                             \
        free(darray->data); darray->data = NULL;                                \
        darray->capacity = darray->size = 0;                                    \
        return DARRAY_SUCCESS;                                                  \
    }                                                                           \
                                                                                \
    DARRAY_ func_tag##_resize(struct_tag *darray, size_t new_capacity) {        \
        if (darray->capacity == new_capacity)                                   \
            return DARRAY_SUCCESS;                                              \
                                                                                \
        void *temp = realloc(darray->data, new_capacity * sizeof(data_type));   \
        if (temp == NULL)                                                       \
            return DARRAY_ALLOC_FAILURE;                                        \
                                                                                \
        darray->data = (data_type *)temp;                                       \
        if (darray->size > new_capacity)                                        \
            darray->size = new_capacity;                                        \
        darray->capacity = new_capacity;                                        \
        return DARRAY_SUCCESS;                                                  \
    }                                                                           \
                                                                                \
    DARRAY_ func_tag##_push(struct_tag *darray, data_type new_elem) {           \
        if (darray->size >= darray->capacity) {                                 \
            size_t new_capacity = darray->capacity ? darray->capacity << 1 : 4; \
            DARRAY_ result = func_tag##_resize(darray, new_capacity);           \
            if (result != DARRAY_SUCCESS)                                       \
                return result;                                                  \
        }                                                                       \
                                                                                \
        memcpy(&darray->data[darray->size++], &new_elem, sizeof(data_type));    \
        return DARRAY_SUCCESS;                                                  \
    }                                                                           \
                                                                                \
    DARRAY_ func_tag##_pop(struct_tag *darray, data_type *_elem) {              \
        if (darray->size <= 0)                                                  \
            return DARRAY_ON_EMPTY;                                             \
                                                                                \
        darray->size--;                                                         \
        if (_elem != NULL)                                                      \
            memcpy(_elem, &darray->data[darray->size], sizeof(data_type));      \
        return DARRAY_SUCCESS;                                                  \
    }                                                                           \
                                                                                \
    DARRAY_ func_tag##_pack(struct_tag *darray) {                               \
        return func_tag##_resize(darray, darray->size);                         \
    }

#endif  // _DARRAY_H