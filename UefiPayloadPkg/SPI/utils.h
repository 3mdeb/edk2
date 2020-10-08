#ifndef UTILS_H
#define UTILS_H

#define ALIGN(x,a)              __ALIGN_MASK(x,(typeof(x))(a)-1UL)
#define __ALIGN_MASK(x,mask)    (((x)+(mask))&~(mask))
#define ALIGN_UP(x,a)           ALIGN((x),(a))
#define ALIGN_DOWN(x,a)         ((x) & ~((typeof(x))(a)-1UL))
#define IS_ALIGNED(x,a)         (((x) & ((typeof(x))(a)-1UL)) == 0)

#define KiB (1<<10)

#endif /* UTILS_H */
