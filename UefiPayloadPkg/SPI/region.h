/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _REGION_H_
#define _REGION_H_


#include <Include/PiDxe.h>

/*
 * Region support.
 *
 * Regions are intended to abstract away the access mechanisms for blocks of
 * data. This could be SPI, eMMC, or a memory region as the backing store.
 * They are accessed through a region_device.  Subregions can be made by
 * chaining together multiple region_devices.
 */

struct region_device;

/*
 * Returns NULL on error otherwise a buffer is returned with the contents of
 * the requested data at offset of size.
 */
void *rdev_mmap(const struct region_device *rd, __SIZE_TYPE__ offset, __SIZE_TYPE__ size);

/* Unmap a previously mapped area. Returns 0 on success, < 0 on error. */
int rdev_munmap(const struct region_device *rd, void *mapping);

/*
 * Returns < 0 on error otherwise returns size of data read at provided
 * offset filling in the buffer passed.
 */
__SIZE_TYPE__ rdev_readat(const struct region_device *rd, void *b, __SIZE_TYPE__ offset,
			__SIZE_TYPE__ size);

/*
 * Returns < 0 on error otherwise returns size of data wrote at provided
 * offset from the buffer passed.
 */
__SIZE_TYPE__ rdev_writeat(const struct region_device *rd, const void *b,
			__SIZE_TYPE__ offset, __SIZE_TYPE__ size);

/*
 * Returns < 0 on error otherwise returns size of data erased.
 * If eraseat ops is not defined it returns size which indicates
 * that operation was successful.
 */
__SIZE_TYPE__ rdev_eraseat(const struct region_device *rd, __SIZE_TYPE__ offset,
			__SIZE_TYPE__ size);

/****************************************
 *  Implementation of a region device   *
 ****************************************/

/*
 * Create a child region of the parent provided the sub-region is within
 * the parent's region. Returns < 0 on error otherwise 0 on success. Note
 * that the child device only calls through the parent's operations.
 */
int rdev_chain(struct region_device *child, const struct region_device *parent,
		__SIZE_TYPE__ offset, __SIZE_TYPE__ size);

/* A region_device operations. */
struct region_device_ops {
	void *(*mmap)(const struct region_device *, __SIZE_TYPE__, __SIZE_TYPE__);
	int (*munmap)(const struct region_device *, void *);
	__SIZE_TYPE__ (*readat)(const struct region_device *, void *, __SIZE_TYPE__, __SIZE_TYPE__);
	__SIZE_TYPE__ (*writeat)(const struct region_device *, const void *, __SIZE_TYPE__,
		__SIZE_TYPE__);
	__SIZE_TYPE__ (*eraseat)(const struct region_device *, __SIZE_TYPE__, __SIZE_TYPE__);
};

struct region {
	__SIZE_TYPE__ offset;
	__SIZE_TYPE__ size;
};

struct region_device {
	const struct region_device *root;
	const struct region_device_ops *ops;
	struct region region;
};

#define REGION_DEV_INIT(ops_, offset_, size_)		\
	{						\
		.root = NULL,				\
		.ops = (ops_),				\
		.region = {				\
			.offset = (offset_),		\
			.size = (size_),		\
		},					\
	}

/* Helper to dynamically initialize region device. */
void region_device_init(struct region_device *rdev,
			const struct region_device_ops *ops, __SIZE_TYPE__ offset,
			__SIZE_TYPE__ size);

/* Return 1 if child is subregion of parent, else 0. */
int region_is_subregion(const struct region *p, const struct region *c);

static inline __SIZE_TYPE__ region_offset(const struct region *r)
{
	return r->offset;
}

static inline __SIZE_TYPE__ region_sz(const struct region *r)
{
	return r->size;
}

static inline __SIZE_TYPE__ region_end(const struct region *r)
{
	return region_offset(r) + region_sz(r);
}

static inline BOOLEAN region_overlap(const struct region *r1, const struct region *r2)
{
	return (region_end(r1) > region_offset(r2)) &&
	       (region_offset(r1) < region_end(r2));
}

static inline const struct region *region_device_region(
					const struct region_device *rdev)
{
	return &rdev->region;
}

static inline __SIZE_TYPE__ region_device_sz(const struct region_device *rdev)
{
	return region_sz(region_device_region(rdev));
}

static inline __SIZE_TYPE__ region_device_offset(const struct region_device *rdev)
{
	return region_offset(region_device_region(rdev));
}

static inline __SIZE_TYPE__ region_device_end(const struct region_device *rdev)
{
	return region_end(region_device_region(rdev));
}

/* Memory map entire region device. Same semantics as rdev_mmap() above. */
static inline void *rdev_mmap_full(const struct region_device *rd)
{
	return rdev_mmap(rd, 0, region_device_sz(rd));
}

static inline int rdev_chain_full(struct region_device *child,
				const struct region_device *parent)
{
	/* Chain full size of parent. */
	return rdev_chain(child, parent, 0, region_device_sz(parent));
}

/*
 * Compute relative offset of the child (c) w.r.t. the parent (p). Returns < 0
 * when child is not within the parent's region.
 */
__SIZE_TYPE__ rdev_relative_offset(const struct region_device *p,
				const struct region_device *c);

struct mem_region_device {
	char *base;
	struct region_device rdev;
};

/* Initialize at runtime a mem_region_device. This would be used when
 * the base and size are dynamic or can't be known during linking.
 * There are two variants: read-only and read-write. */
void mem_region_device_ro_init(struct mem_region_device *mdev, void *base,
				__SIZE_TYPE__ size);

void mem_region_device_rw_init(struct mem_region_device *mdev, void *base,
				__SIZE_TYPE__ size);

extern const struct region_device_ops mem_rdev_ro_ops;

extern const struct region_device_ops mem_rdev_rw_ops;

/* Statically initialize mem_region_device. */
#define MEM_REGION_DEV_INIT(base_, size_, ops_)				\
	{								\
		.base = (void *)(base_),				\
		.rdev = REGION_DEV_INIT((ops_), 0, (size_)),		\
	}

#define MEM_REGION_DEV_RO_INIT(base_, size_)				\
		MEM_REGION_DEV_INIT(base_, size_, &mem_rdev_ro_ops)	\

#define MEM_REGION_DEV_RW_INIT(base_, size_)				\
		MEM_REGION_DEV_INIT(base_, size_, &mem_rdev_rw_ops)	\

struct mem_pool {
	UINT8 *buf;
	__SIZE_TYPE__ size;
	UINT8 *last_alloc;
	__SIZE_TYPE__ free_offset;
};

struct mmap_helper_region_device {
	struct mem_pool pool;
	struct region_device rdev;
};

#define MMAP_HELPER_REGION_INIT(ops_, offset_, size_)			\
	{								\
		.rdev = REGION_DEV_INIT((ops_), (offset_), (size_)),	\
	}

void mmap_helper_device_init(struct mmap_helper_region_device *mdev,
				void *cache, __SIZE_TYPE__ cache_size);

void *mmap_helper_rdev_mmap(const struct region_device *, __SIZE_TYPE__, __SIZE_TYPE__);
int mmap_helper_rdev_munmap(const struct region_device *, void *);

/* A translated region device provides the ability to publish a region device
 * in one address space and use an access mechanism within another address
 * space. The sub region is the window within the 1st address space and
 * the request is modified prior to accessing the second address space
 * provided by access_dev. */
struct xlate_region_device {
	const struct region_device *access_dev;
	struct region sub_region;
	struct region_device rdev;
};

extern const struct region_device_ops xlate_rdev_ro_ops;

extern const struct region_device_ops xlate_rdev_rw_ops;

#define XLATE_REGION_DEV_INIT(access_dev_, sub_offset_, sub_size_,	\
		parent_sz_, ops_)					\
	{								\
		.access_dev = access_dev_,				\
		.sub_region = {						\
			.offset = (sub_offset_),			\
			.size = (sub_size_),				\
		},							\
		.rdev = REGION_DEV_INIT((ops_), 0,  (parent_sz_)),	\
	}

#define XLATE_REGION_DEV_RO_INIT(access_dev_, sub_offset_, sub_size_,	\
		parent_sz_)						\
		XLATE_REGION_DEV_INIT(access_dev_, sub_offset_,		\
			sub_size_, parent_sz_, &xlate_rdev_ro_ops),	\

#define XLATE_REGION_DEV_RW_INIT(access_dev_, sub_offset_, sub_size_,	\
		parent_sz_)						\
		XLATE_REGION_DEV_INIT(access_dev_, sub_offset_,		\
			sub_size_, parent_sz_, &xlate_rdev_rw_ops),	\

/* Helper to dynamically initialize xlate region device. */
void xlate_region_device_ro_init(struct xlate_region_device *xdev,
			      const struct region_device *access_dev,
			      __SIZE_TYPE__ sub_offset, __SIZE_TYPE__ sub_size,
			      __SIZE_TYPE__ parent_size);

void xlate_region_device_rw_init(struct xlate_region_device *xdev,
			      const struct region_device *access_dev,
			      __SIZE_TYPE__ sub_offset, __SIZE_TYPE__ sub_size,
			      __SIZE_TYPE__ parent_size);

/* This type can be used for incoherent access where the read and write
 * operations are backed by separate drivers. An example is x86 systems
 * with memory mapped media for reading but use a spi flash driver for
 * writing. One needs to ensure using this object is appropriate in context. */
struct incoherent_rdev {
	struct region_device rdev;
	const struct region_device *read;
	const struct region_device *write;
};

/* Initialize an incoherent_rdev based on the region as well as the read and
 * write rdevs. The read and write rdevs should match in size to the passed
 * in region. If not the initialization will fail returning NULL. Otherwise
 * the function will return a pointer to the containing region_device to
 * be used for region operations. Therefore, the lifetime of the returned
 * pointer matches the lifetime of the incoherent_rdev object. Likewise,
 * the lifetime of the read and write rdev need to match the lifetime of
 * the incoherent_rdev object. */
const struct region_device *incoherent_rdev_init(struct incoherent_rdev *irdev,
				const struct region *r,
				const struct region_device *read,
				const struct region_device *write);

#endif /* _REGION_H_ */
