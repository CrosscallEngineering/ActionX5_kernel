/*
 * Copyright (C) 2019 Hisense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/version.h>

static char *resource_reclaim = ".apk .dex .jar .odex";

/* Below function is same as "madvise_dontneed() in mm/madvise.c */
static long shrinker_madvise_dontneed(struct vm_area_struct * vma,
			     unsigned long start, unsigned long end)
{
	if (vma->vm_flags & (VM_LOCKED|VM_HUGETLB|VM_PFNMAP))
		return -EINVAL;

	zap_page_range(vma, start, end - start);

	return 0;
}

const char *get_path_ext(const char *path)
{
	const char *dot = NULL;
	if (path == NULL) {
		return NULL;
	}

	dot = strrchr(path, '.');
	if (!dot || dot == path)
		return NULL;

	return dot + 1;
}

static bool is_useless_resource_vma(struct vm_area_struct *vma)
{
	const char *path;
	const char* suffix;
	bool ret = false;

	if (!vma->vm_file) {
		goto out;
	}

	if (vma->anon_vma) {
		/*
		 * For some readonly or rw vma, a file mapping that has
		 * had some COW done. Since pages might have been
		 * written, if free, the data will loss
		*/
		goto out;
	}

	path = vma->vm_file->f_path.dentry->d_name.name;
	suffix = get_path_ext(path);
	if (suffix != NULL && strstr(resource_reclaim, suffix)) {
		pr_err("reclaim vma path=%s\n", path);
		ret = true;
	}

out:
	return ret;
}

void useless_resource_shrinker(struct mm_struct *mm)
{
	struct vm_area_struct *vma;

	down_read(&mm->mmap_sem);
	for (vma = mm->mmap ; vma; vma = vma->vm_next) {
		if (is_useless_resource_vma(vma))
			shrinker_madvise_dontneed(vma,
					vma->vm_start , vma->vm_end);
	}
	up_read(&mm->mmap_sem);
}

