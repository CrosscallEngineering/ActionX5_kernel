/*
 * fs/sdcardfs/main.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfs.h"
#include <linux/fscrypt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>
#ifdef CONFIG_MULTI_USER_ID
#include <linux/his_debug_base.h>
#include <linux/acpi.h>
#endif/*#ifdef CONFIG_MULTI_USER_ID*/

#if defined(CONFIG_SDCARD_FS_BYPASS_PERM_CHECK)
#include <linux/proc_fs.h>
#endif

enum {
	Opt_fsuid,
	Opt_fsgid,
	Opt_gid,
	Opt_debug,
	Opt_mask,
	Opt_multiuser,
	Opt_userid,
	Opt_reserved_mb,
	Opt_gid_derivation,
	Opt_default_normal,
	Opt_nocache,
	Opt_unshared_obb,
	Opt_err,
};

static const match_table_t sdcardfs_tokens = {
	{Opt_fsuid, "fsuid=%u"},
	{Opt_fsgid, "fsgid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_debug, "debug"},
	{Opt_mask, "mask=%u"},
	{Opt_userid, "userid=%d"},
	{Opt_multiuser, "multiuser"},
	{Opt_gid_derivation, "derive_gid"},
	{Opt_default_normal, "default_normal"},
	{Opt_unshared_obb, "unshared_obb"},
	{Opt_reserved_mb, "reserved_mb=%u"},
	{Opt_nocache, "nocache"},
	{Opt_err, NULL}
};

static int parse_options(struct super_block *sb, char *options, int silent,
				int *debug, struct sdcardfs_vfsmount_options *vfsopts,
				struct sdcardfs_mount_options *opts)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;

	/* by default, we use AID_MEDIA_RW as uid, gid */
	opts->fs_low_uid = AID_MEDIA_RW;
	opts->fs_low_gid = AID_MEDIA_RW;
	vfsopts->mask = 0;
	opts->multiuser = false;
	opts->fs_user_id = 0;
	vfsopts->gid = 0;
	/* by default, 0MB is reserved */
	opts->reserved_mb = 0;
	/* by default, gid derivation is off */
	opts->gid_derivation = false;
	opts->default_normal = false;
	opts->nocache = false;

	*debug = 0;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;

		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);

		switch (token) {
		case Opt_debug:
			*debug = 1;
			break;
		case Opt_fsuid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_uid = option;
			break;
		case Opt_fsgid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_gid = option;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->gid = option;
			break;
		case Opt_userid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_user_id = option;
			break;
		case Opt_mask:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->mask = option;
			break;
		case Opt_multiuser:
			opts->multiuser = true;
			break;
		case Opt_reserved_mb:
			if (match_int(&args[0], &option))
				return 0;
			opts->reserved_mb = option;
			break;
		case Opt_gid_derivation:
			opts->gid_derivation = true;
			break;
		case Opt_default_normal:
			opts->default_normal = true;
			break;
		case Opt_nocache:
			opts->nocache = true;
			break;
		case Opt_unshared_obb:
			opts->unshared_obb = true;
			break;
		/* unknown option */
		default:
			if (!silent)
				pr_err("Unrecognized mount option \"%s\" or missing value", p);
			return -EINVAL;
		}
	}

	if (*debug) {
		pr_info("sdcardfs : options - debug:%d\n", *debug);
		pr_info("sdcardfs : options - uid:%d\n",
							opts->fs_low_uid);
		pr_info("sdcardfs : options - gid:%d\n",
							opts->fs_low_gid);
	}

	return 0;
}

int parse_options_remount(struct super_block *sb, char *options, int silent,
				struct sdcardfs_vfsmount_options *vfsopts)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;
	int debug;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;

		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);

		switch (token) {
		case Opt_debug:
			debug = 1;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->gid = option;

			break;
		case Opt_mask:
			if (match_int(&args[0], &option))
				return 0;
			vfsopts->mask = option;
			break;
		case Opt_unshared_obb:
		case Opt_default_normal:
		case Opt_multiuser:
		case Opt_userid:
		case Opt_fsuid:
		case Opt_fsgid:
		case Opt_reserved_mb:
			pr_warn("Option \"%s\" can't be changed during remount\n", p);
		case Opt_gid_derivation:
			if (!silent)
				pr_warn("Option \"%s\" can't be changed during remount\n", p);
			break;
		/* unknown option */
		default:
			if (!silent)
				pr_err("Unrecognized mount option \"%s\" or missing value", p);
			return -EINVAL;
		}
	}

	if (debug) {
		pr_info("sdcardfs : options - debug:%d\n", debug);
		pr_info("sdcardfs : options - gid:%d\n", vfsopts->gid);
		pr_info("sdcardfs : options - mask:%d\n", vfsopts->mask);
	}

	return 0;
}

#if 0
/*
 * our custom d_alloc_root work-alike
 *
 * we can't use d_alloc_root if we want to use our own interpose function
 * unchanged, so we simply call our own "fake" d_alloc_root
 */
static struct dentry *sdcardfs_d_alloc_root(struct super_block *sb)
{
	struct dentry *ret = NULL;

	if (sb) {
		static const struct qstr name = {
			.name = "/",
			.len = 1
		};

		ret = d_alloc(NULL, &name);
		if (ret) {
			d_set_d_op(ret, &sdcardfs_ci_dops);
			ret->d_sb = sb;
			ret->d_parent = ret;
		}
	}
	return ret;
}
#endif

DEFINE_MUTEX(sdcardfs_super_list_lock);
EXPORT_SYMBOL_GPL(sdcardfs_super_list_lock);
LIST_HEAD(sdcardfs_super_list);
EXPORT_SYMBOL_GPL(sdcardfs_super_list);

/*
 * There is no need to lock the sdcardfs_super_info's rwsem as there is no
 * way anyone can have a reference to the superblock at this point in time.
 */
static int sdcardfs_read_super(struct vfsmount *mnt, struct super_block *sb,
		const char *dev_name, void *raw_data, int silent)
{
	int err = 0;
	int debug;
	struct super_block *lower_sb;
	struct path lower_path;
	struct sdcardfs_sb_info *sb_info;
	struct sdcardfs_vfsmount_options *mnt_opt = mnt->data;
	struct inode *inode;

	pr_info("sdcardfs version 2.0\n");

	if (!dev_name) {
		pr_err("sdcardfs: read_super: missing dev_name argument\n");
		err = -EINVAL;
		goto out;
	}

	pr_info("sdcardfs: dev_name -> %s\n", dev_name);
	pr_info("sdcardfs: options -> %s\n", (char *)raw_data);
	pr_info("sdcardfs: mnt -> %p\n", mnt);

	/* parse lower path */
	err = kern_path(dev_name, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
			&lower_path);
	if (err) {
		pr_err("sdcardfs: error accessing lower directory '%s'\n", dev_name);
		goto out;
	}

	/* allocate superblock private data */
	sb->s_fs_info = kzalloc(sizeof(struct sdcardfs_sb_info), GFP_KERNEL);
	if (!SDCARDFS_SB(sb)) {
		pr_crit("sdcardfs: read_super: out of memory\n");
		err = -ENOMEM;
		goto out_free;
	}

	sb_info = sb->s_fs_info;
	/* parse options */
	err = parse_options(sb, raw_data, silent, &debug, mnt_opt, &sb_info->options);
	if (err) {
		pr_err("sdcardfs: invalid options\n");
		goto out_freesbi;
	}

	/* set the lower superblock field of upper superblock */
	lower_sb = lower_path.dentry->d_sb;
	atomic_inc(&lower_sb->s_active);
	sdcardfs_set_lower_super(sb, lower_sb);

	sb->s_stack_depth = lower_sb->s_stack_depth + 1;
	if (sb->s_stack_depth > FILESYSTEM_MAX_STACK_DEPTH) {
		pr_err("sdcardfs: maximum fs stacking depth exceeded\n");
		err = -EINVAL;
		goto out_sput;
	}

	/* inherit maxbytes from lower file system */
	sb->s_maxbytes = lower_sb->s_maxbytes;

	/*
	 * Our c/m/atime granularity is 1 ns because we may stack on file
	 * systems whose granularity is as good.
	 */
	sb->s_time_gran = 1;

	sb->s_magic = SDCARDFS_SUPER_MAGIC;
	sb->s_op = &sdcardfs_sops;

	/* get a new inode and allocate our root dentry */
	inode = sdcardfs_iget(sb, d_inode(lower_path.dentry), 0);
	if (IS_ERR(inode)) {
		err = PTR_ERR(inode);
		goto out_sput;
	}
	sb->s_root = d_make_root(inode);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_sput;
	}
	d_set_d_op(sb->s_root, &sdcardfs_ci_dops);

	/* link the upper and lower dentries */
	sb->s_root->d_fsdata = NULL;
	err = new_dentry_private_data(sb->s_root);
	if (err)
		goto out_freeroot;

	/* set the lower dentries for s_root */
	sdcardfs_set_lower_path(sb->s_root, &lower_path);

	/*
	 * No need to call interpose because we already have a positive
	 * dentry, which was instantiated by d_make_root.  Just need to
	 * d_rehash it.
	 */
	d_rehash(sb->s_root);

	/* setup permission policy */
	sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
	mutex_lock(&sdcardfs_super_list_lock);
	if (sb_info->options.multiuser) {
		setup_derived_state(d_inode(sb->s_root), PERM_PRE_ROOT,
				sb_info->options.fs_user_id, AID_ROOT);
		snprintf(sb_info->obbpath_s, PATH_MAX, "%s/obb", dev_name);
	} else {
		setup_derived_state(d_inode(sb->s_root), PERM_ROOT,
				sb_info->options.fs_user_id, AID_ROOT);
		snprintf(sb_info->obbpath_s, PATH_MAX, "%s/Android/obb", dev_name);
	}
	fixup_tmp_permissions(d_inode(sb->s_root));
	sb_info->sb = sb;
	list_add(&sb_info->list, &sdcardfs_super_list);
	mutex_unlock(&sdcardfs_super_list_lock);

	sb_info->fscrypt_nb.notifier_call = sdcardfs_on_fscrypt_key_removed;
	fscrypt_register_key_removal_notifier(&sb_info->fscrypt_nb);

	if (!silent)
		pr_info("sdcardfs: mounted on top of %s type %s\n",
				dev_name, lower_sb->s_type->name);
	goto out; /* all is well */

	/* no longer needed: free_dentry_private_data(sb->s_root); */
out_freeroot:
	dput(sb->s_root);
	sb->s_root = NULL;
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
out_freesbi:
	kfree(SDCARDFS_SB(sb));
	sb->s_fs_info = NULL;
out_free:
	path_put(&lower_path);

out:
	return err;
}

struct sdcardfs_mount_private {
	struct vfsmount *mnt;
	const char *dev_name;
	void *raw_data;
};

static int __sdcardfs_fill_super(
	struct super_block *sb,
	void *_priv, int silent)
{
	struct sdcardfs_mount_private *priv = _priv;

	return sdcardfs_read_super(priv->mnt,
		sb, priv->dev_name, priv->raw_data, silent);
}

static struct dentry *sdcardfs_mount(struct vfsmount *mnt,
		struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	struct sdcardfs_mount_private priv = {
		.mnt = mnt,
		.dev_name = dev_name,
		.raw_data = raw_data
	};

	return mount_nodev(fs_type, flags,
		&priv, __sdcardfs_fill_super);
}

static struct dentry *sdcardfs_mount_wrn(struct file_system_type *fs_type,
		    int flags, const char *dev_name, void *raw_data)
{
	WARN(1, "sdcardfs does not support mount. Use mount2.\n");
	return ERR_PTR(-EINVAL);
}

void *sdcardfs_alloc_mnt_data(void)
{
	return kmalloc(sizeof(struct sdcardfs_vfsmount_options), GFP_KERNEL);
}

void sdcardfs_kill_sb(struct super_block *sb)
{
	struct sdcardfs_sb_info *sbi;

	if (sb->s_magic == SDCARDFS_SUPER_MAGIC && sb->s_fs_info) {
		sbi = SDCARDFS_SB(sb);

		fscrypt_unregister_key_removal_notifier(&sbi->fscrypt_nb);

		mutex_lock(&sdcardfs_super_list_lock);
		list_del(&sbi->list);
		mutex_unlock(&sdcardfs_super_list_lock);
	}
	kill_anon_super(sb);
}

static struct file_system_type sdcardfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= SDCARDFS_NAME,
	.mount		= sdcardfs_mount_wrn,
	.mount2		= sdcardfs_mount,
	.alloc_mnt_data = sdcardfs_alloc_mnt_data,
	.kill_sb	= sdcardfs_kill_sb,
	.fs_flags	= 0,
};
MODULE_ALIAS_FS(SDCARDFS_NAME);

#ifdef CONFIG_MULTI_USER_ID
unsigned long  app_divid_id = 881;
bool app_divid_node_be_set = false;
#define MIN_DIVID_ID	881

static ssize_t app_divid_id_proc_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *ppos)
{
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(buffer, count, 10, &value);
	if (ret < 0) {
		printk("illegal value in app_divid_id buffer\n");
		return -EINVAL;
	}

	//printk("app_divid_id buffer=%s, count=%d, value=%lu \n", (char *)buffer, (int)count, value);
	if(value < MIN_DIVID_ID){
		printk("new app_divid_id is out of range: %lu \n", value);
		return -EINVAL;
	}
	app_divid_id = value;
        app_divid_node_be_set = true;
	return count;
}
static int app_divid_id_proc_show(struct seq_file *m, void *v)
{

	/*
	 * Tagged format, for easy grepping and expansion.
	 */
	seq_printf(m, "app divid id is:       %lu\n", app_divid_id);

	return 0;
}

static int app_divid_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, app_divid_id_proc_show, NULL);
}

static const struct file_operations app_divid_id_proc_fops = {
	.open           = app_divid_id_proc_open,
	.read           = seq_read,
	.write          =app_divid_id_proc_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif/*CONFIG_MULTI_USER_ID*/

// ===============================================================================
#if defined(CONFIG_SDCARD_FS_BYPASS_PERM_CHECK)

static struct sdcardfs_bypass_perm_info sdcard_bypass_info;
static struct proc_dir_entry* bypass_procfs_root = NULL;

static void bypass_perm_init()
{
	sdcard_bypass_info.size = 0;
}

bool is_bypass_uid(uid_t app_uid)
{
	int i;
	int size = sdcard_bypass_info.size;

	uid_t* uid = &(sdcard_bypass_info.uids[0]);
	for (i = 0; i < size; i++) {
		if (*uid == app_uid) return true;
		uid++;
	}

	return false;
}

static void add_bypass_uid(uid_t uid)
{
	unsigned int index;

	if (uid == 0) {
		bypass_perm_init();
		return;
	}

	index = sdcard_bypass_info.size;
	if (index >= 8) {
		pr_info("Failed to add bypass uid, max size is 8\n");
		return;
	}
	sdcard_bypass_info.uids[index++] = uid;
	sdcard_bypass_info.size = index;
}

static struct proc_dir_entry* hx_create_procfs_root(void)
{
	static int create_ok = 0;

	if (create_ok == 1)
		goto out;

	bypass_procfs_root = proc_mkdir("storage_control", NULL);
	if (IS_ERR(bypass_procfs_root))
		return NULL;

	create_ok = 1;
out:
	return bypass_procfs_root;
}

static int hx_create_procfs_file(const char *name, umode_t mode, const struct file_operations *fops)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *procfs_root;

	procfs_root = hx_create_procfs_root();
	if (procfs_root == NULL)
		return -ENOMEM;

	entry = proc_create(name, mode, procfs_root, fops);
	if (!entry)
		return -ENOMEM;

	return 0;
}

static ssize_t bypass_perm_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int value;
	int ret;

	ret = kstrtouint_from_user(buffer, count, 10, &value);
	if (ret < 0) {
		pr_info("illegal value in bypass_uids buffer\n");
		return -EINVAL;
	}
	pr_info("bypass_perm_proc_write add uid=%d\n", value);

	add_bypass_uid(value);
	return count;
}

static int bypass_perm_proc_show(struct seq_file *m, void *v)
{
	int i;
	int size = sdcard_bypass_info.size;

	seq_printf(m, "bypass-uids size: %d\n", size);
	for (i = 0; i < size; i++) {
		seq_printf(m, "bypass-uid[%d]=%d\n", i, sdcard_bypass_info.uids[i]);
	}
	return 0;
}

static int bypass_perm_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bypass_perm_proc_show, NULL);
}

static const struct file_operations bypass_perm_proc_fops = {
	.open           = bypass_perm_proc_open,
	.read           = seq_read,
	.write          = bypass_perm_proc_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

#endif // CONFIG_SDCARD_FS_BYPASS_PERM_CHECK
// ===============================================================================

static int __init init_sdcardfs_fs(void)
{
	int err;

	#ifdef CONFIG_MULTI_USER_ID
	int ret = -EPERM;
	ret = his_create_procfs_file("app_divid_id", S_IRWXUGO, &app_divid_id_proc_fops);
	if (ret < 0)
		printk("proc create app_divid_id failed !\n");
	#endif/*CONFIG_MULTI_USER_ID*/
	
	pr_info("Registering sdcardfs " SDCARDFS_VERSION "\n");

#if defined(CONFIG_SDCARD_FS_BYPASS_PERM_CHECK)
	bypass_perm_init();

	{
		int ret = -EPERM;
		ret = hx_create_procfs_file("bypass_uids", S_IRUSR|S_IWUSR, &bypass_perm_proc_fops);
		if (ret < 0) {
			pr_info("Failed to create bypass proc node\n");
		}
	}
#endif

	err = sdcardfs_init_inode_cache();
	if (err)
		goto out;
	err = sdcardfs_init_dentry_cache();
	if (err)
		goto out;
	err = packagelist_init();
	if (err)
		goto out;
	err = register_filesystem(&sdcardfs_fs_type);
out:
	if (err) {
		sdcardfs_destroy_inode_cache();
		sdcardfs_destroy_dentry_cache();
		packagelist_exit();
	}
	return err;
}

static void __exit exit_sdcardfs_fs(void)
{
	sdcardfs_destroy_inode_cache();
	sdcardfs_destroy_dentry_cache();
	packagelist_exit();
	unregister_filesystem(&sdcardfs_fs_type);
	pr_info("Completed sdcardfs module unload\n");
}

/* Original wrapfs authors */
MODULE_AUTHOR("Erez Zadok, Filesystems and Storage Lab, Stony Brook University (http://www.fsl.cs.sunysb.edu/)");

/* Original sdcardfs authors */
MODULE_AUTHOR("Woojoong Lee, Daeho Jeong, Kitae Lee, Yeongjin Gil System Memory Lab., Samsung Electronics");

/* Current maintainer */
MODULE_AUTHOR("Daniel Rosenberg, Google");
MODULE_DESCRIPTION("Sdcardfs " SDCARDFS_VERSION);
MODULE_LICENSE("GPL");

module_init(init_sdcardfs_fs);
module_exit(exit_sdcardfs_fs);
