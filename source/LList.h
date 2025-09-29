#if !defined(LLIST_H)
#define LLIST_H

#include <string.h>
#include <stdlib.h>

typedef enum {
    LLIST_SUCCESS,
    LLIST_ON_EMPTY,
    LLIST_ALLOC_FAILURE,
} LLIST_;

#define LLIST_DEFINE(struct_tag, func_tag, data_type)                       \
    typedef struct struct_tag {                                             \
        struct struct_tag *next, *prev;                                     \
        data_type data;                                                     \
    } struct_tag;                                                           \
                                                                            \
    struct_tag *func_tag##_new_root(data_type new_data) {                   \
        struct_tag *new_node = (struct_tag *)malloc(sizeof(struct_tag));    \
        if (new_node == NULL)                                               \
            return new_node;                                                \
                                                                            \
        new_node->next = new_node->prev = new_node;                         \
        memcpy(&new_node->data, &new_data, sizeof(data_type));              \
        return new_node;                                                    \
    }                                                                       \
                                                                            \
    size_t func_tag##_size(struct_tag *root) {                              \
        if (root == NULL)                                                   \
            return 0;                                                       \
                                                                            \
        size_t size = 0;                                                    \
        struct_tag *temp = root;                                            \
        do {                                                                \
            size++;                                                         \
            temp = temp->next;                                              \
        } while (temp != root);                                             \
                                                                            \
        return size;                                                        \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_insert(struct_tag *pred, data_type new_data) {        \
        if (pred == NULL)                                                   \
            return LLIST_ON_EMPTY;                                          \
                                                                            \
        struct_tag *new_node = func_tag##_new_root(new_data);               \
        if (new_node == NULL)                                               \
            return LLIST_ALLOC_FAILURE;                                     \
                                                                            \
        struct_tag *next = pred->next;                                      \
        pred->next = new_node;                                              \
        new_node->next = next;                                              \
        new_node->prev = pred;                                              \
        next->prev = new_node;                                              \
        return LLIST_SUCCESS;                                               \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_remove(struct_tag **_link) {                          \
        if (*_link == NULL)                                                 \
            return LLIST_ON_EMPTY;                                          \
                                                                            \
        if (*_link == (*_link)->next) {                                     \
            free(*_link); *_link = NULL;                                    \
            return LLIST_SUCCESS;                                           \
        }                                                                   \
                                                                            \
        struct_tag *prev = (*_link)->prev;                                  \
        struct_tag *next = (*_link)->next;                                  \
        free(*_link); *_link = NULL;                                        \
        prev->next = next;                                                  \
        next->prev = prev;                                                  \
        return LLIST_SUCCESS;                                               \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_push_back(struct_tag **_root, data_type new_data) {   \
        if (*_root == NULL) {                                               \
            *_root = func_tag##_new_root(new_data);                         \
            if (*_root == NULL)                                             \
                return LLIST_ALLOC_FAILURE;                                 \
            return LLIST_SUCCESS;                                           \
        }                                                                   \
                                                                            \
        return func_tag##_insert((*_root)->prev, new_data);                 \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_push_front(struct_tag **_root, data_type new_data) {  \
        if (*_root == NULL) {                                               \
            *_root = func_tag##_new_root(new_data);                         \
            if (*_root == NULL)                                             \
                return LLIST_ALLOC_FAILURE;                                 \
            return LLIST_SUCCESS;                                           \
        }                                                                   \
                                                                            \
        LLIST_ result = func_tag##_insert((*_root)->prev, new_data);        \
        if (result != LLIST_SUCCESS)                                        \
            return result;                                                  \
        *_root = (*_root)->prev;                                            \
        return LLIST_SUCCESS;                                               \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_pop_back(struct_tag **_root) {                        \
        return func_tag##_remove(&(*_root)->prev);                          \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_pop_front(struct_tag **_root) {                       \
        if (*_root == NULL)                                                 \
            return LLIST_ON_EMPTY;                                          \
                                                                            \
        if (*_root == (*_root)->next) {                                     \
            free(*_root); *_root = NULL;                                    \
            return LLIST_SUCCESS;                                           \
        }                                                                   \
                                                                            \
        struct_tag *next = (*_root)->next;                                  \
        func_tag##_remove(_root);                                           \
        *_root = next;                                                      \
        return LLIST_SUCCESS;                                               \
    }                                                                       \
                                                                            \
    LLIST_ func_tag##_delete(struct_tag **_root) {                          \
        if (*_root == NULL)                                                 \
            return LLIST_ON_EMPTY;                                          \
                                                                            \
        if (*_root == (*_root)->next) {                                     \
            free(*_root); *_root = NULL;                                    \
            return LLIST_SUCCESS;                                           \
        }                                                                   \
        struct_tag *temp = *_root;                                          \
        do {                                                                \
            struct_tag *next = temp->next;                                  \
            free(temp);                                                     \
            temp = next;                                                    \
        } while (temp != *_root);                                           \
                                                                            \
        *_root = NULL;                                                      \
        return LLIST_SUCCESS;                                               \
    }

#endif  // LLIST_H