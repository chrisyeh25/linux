// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DEVICE_NAME			"aspeed-xdma"
#define PROTOTYPE			1

#ifdef PROTOTYPE
#define SCU_STRAP			0x070
#define  SCU_STRAP_VGA_MEM		GENMASK(3, 2)

#define SCU_PCIE_CONF			0x180
#define  SCU_PCIE_CONF_VGA_EN		BIT(0)
#define  SCU_PCIE_CONF_BMC_EN		BIT(8)
#define  SCU_PCIE_CONF_BMC_EN_MMIO	BIT(9)
#define  SCU_PCIE_CONF_BMC_EN_IRQ	BIT(13)
#define  SCU_PCIE_CONF_BMC_EN_DMA	BIT(14)

#define SDMC_CONF			0x004
#define  SDMC_CONF_MEM			GENMASK(1, 0)
#endif /* PROTOTYPE */

#define XDMA_CMD_SIZE			8
#define XDMA_BYTE_ALIGN			16
#define XDMA_NUM_CMDS			2

#define XDMA_CMD_BMC_CHECK		BIT(0)
#define XDMA_CMD_BMC_ADDR		GENMASK(29, 4)
#define XDMA_CMD_BMC_DIR_US		BIT(31)

#define XDMA_CMD_COMM1_HI_HOST_PITCH	GENMASK(14, 3)
#define XDMA_CMD_COMM1_HI_BMC_PITCH	GENMASK(30, 19)

#define XDMA_CMD_CONF_CHECK		BIT(1)
#define XDMA_CMD_CONF_LEN		GENMASK(14, 4)
#define XDMA_CMD_CONF_IRQ_BMC		BIT(15)
#define XDMA_CMD_CONF_FRAME_NO		GENMASK(27, 16)
#define XDMA_CMD_CONF_IRQ		BIT(31)

#define XDMA_DS_PCIE_REQ_SIZE_128	0
#define XDMA_DS_PCIE_REQ_SIZE_256	1
#define XDMA_DS_PCIE_REQ_SIZE_512	2
#define XDMA_DS_PCIE_REQ_SIZE_1K	3
#define XDMA_DS_PCIE_REQ_SIZE_2K	4
#define XDMA_DS_PCIE_REQ_SIZE_4K	5

#define XDMA_BMC_CMD_QUEUE_ADDR		0x10
#define XDMA_BMC_CMD_QUEUE_ENDP		0x14
#define XDMA_BMC_CMD_QUEUE_WRITEP	0x18
#define XDMA_BMC_CMD_QUEUE_READP	0x1c
#define XDMA_CTRL			0x20
#define  XDMA_CTRL_US_COMP		BIT(4)
#define  XDMA_CTRL_DS_COMP		BIT(5)
#define  XDMA_CTRL_DS_DIRTY		BIT(6)
#define  XDMA_CTRL_DS_PCIE_REQ_SIZE	GENMASK(19, 17)
#define  XDMA_CTRL_DS_DATA_TIMEOUT	BIT(28)
#define  XDMA_CTRL_DS_CHECK_ID		BIT(29)
#define XDMA_STATUS			0x24
#define  XDMA_STATUS_US_COMP		BIT(4)
#define  XDMA_STATUS_DS_COMP		BIT(5)

struct aspeed_xdma_cmd {
	u32 host_addr_lo;
	u32 host_addr_hi;
	u32 bmc_addr;
	u32 comm1_hi;
	u32 conf;
	u32 id;
	u32 resv0;
	u32 resv1;
};

struct aspeed_xdma_addr {
	dma_addr_t dma;
	void *virt;
};

struct aspeed_xdma_op {
	bool upstream;
	u32 bmc_addr;
	u64 host_addr;
	u32 len;
};

enum {
	XDMA_IN_PRG,
};

struct aspeed_xdma {
	struct device *dev;
	void __iomem *base;
	struct reset_control *reset;

	unsigned long flags;
	unsigned int cmd_idx;
	struct aspeed_xdma_cmd *cmds;
	struct aspeed_xdma_op *cur_op;
	wait_queue_head_t wait;

#ifdef PROTOTYPE
	u32 vga_size;
/*	struct aspeed_xdma_addr vga; */
	void __iomem *vga;
	struct aspeed_xdma_op op;
	struct miscdevice misc;
#endif /* PROTOTYPE */
};

static u32 aspeed_xdma_read(struct aspeed_xdma *ctx, u32 reg)
{
	u32 v = readl(ctx->base + reg);

	dev_info(ctx->dev, "read %02x[%08x]\n", reg, v);
	return v;
}

static void aspeed_xdma_write(struct aspeed_xdma *ctx, u32 reg, u32 val)
{
	writel(val, ctx->base + reg);
	dev_info(ctx->dev, "write %02x[%08x]\n", reg, readl(ctx->base + reg));
}

static void aspeed_xdma_start(struct aspeed_xdma *ctx,
			      struct aspeed_xdma_op *op)
{
	unsigned int next_idx = (ctx->cmd_idx + 1) % XDMA_NUM_CMDS;
	struct aspeed_xdma_cmd *cmd = &ctx->cmds[ctx->cmd_idx];

	cmd->host_addr_lo = (u32)(op->host_addr & 0xFFFFFFFULL);
	cmd->host_addr_hi = (u32)(op->host_addr >> 32);
	cmd->bmc_addr = (op->bmc_addr & XDMA_CMD_BMC_ADDR) |
		XDMA_CMD_BMC_CHECK | (op->upstream ? XDMA_CMD_BMC_DIR_US : 0);
	cmd->conf = XDMA_CMD_CONF_CHECK |
		FIELD_PREP(XDMA_CMD_CONF_LEN, op->len / XDMA_BYTE_ALIGN) |
		XDMA_CMD_CONF_IRQ_BMC | FIELD_PREP(XDMA_CMD_CONF_FRAME_NO, 1) |
		XDMA_CMD_CONF_IRQ;

	dev_info(ctx->dev, "cmd q %d\n", ctx->cmd_idx);
	dev_info(ctx->dev, "comm0: %08x%08x\n", cmd->host_addr_hi,
		cmd->host_addr_lo);
	dev_info(ctx->dev, "comm1: %08x%08x\n", cmd->comm1_hi, cmd->bmc_addr);
	dev_info(ctx->dev, "comm2: %08x%08x\n", cmd->id, cmd->conf);

	aspeed_xdma_write(ctx, XDMA_BMC_CMD_QUEUE_WRITEP,
			  next_idx * XDMA_CMD_SIZE);
	ctx->cmd_idx = next_idx;
	set_bit(XDMA_IN_PRG, &ctx->flags);

	ctx->cur_op = op;
}

static void aspeed_xdma_done(struct aspeed_xdma *ctx)
{
	clear_bit(XDMA_IN_PRG, &ctx->flags);
	wake_up_interruptible_all(&ctx->wait);

	ctx->cur_op = NULL;
}

static irqreturn_t aspeed_xdma_irq(int irq, void *arg)
{
	struct aspeed_xdma *ctx = arg;
	struct aspeed_xdma_op *op = ctx->cur_op;
	u32 status = aspeed_xdma_read(ctx, XDMA_STATUS);

	if (status & XDMA_STATUS_US_COMP) {
		if (op && op->upstream)
			aspeed_xdma_done(ctx);
	}

	if (status & XDMA_STATUS_DS_COMP) {
		if (op && !op->upstream)
			aspeed_xdma_done(ctx);
	}

	aspeed_xdma_write(ctx, XDMA_STATUS, status);

	return IRQ_HANDLED;
}

static bool aspeed_xdma_init(struct aspeed_xdma *ctx)
{
	const u32 ctrl = XDMA_CTRL_US_COMP | XDMA_CTRL_DS_COMP |
		XDMA_CTRL_DS_DIRTY | FIELD_PREP(XDMA_CTRL_DS_PCIE_REQ_SIZE,
						XDMA_DS_PCIE_REQ_SIZE_256) |
		XDMA_CTRL_DS_DATA_TIMEOUT | XDMA_CTRL_DS_CHECK_ID;
	const size_t size = sizeof(struct aspeed_xdma_cmd) * XDMA_NUM_CMDS;
	unsigned int i;
	phys_addr_t cmdq;

	ctx->cmds = devm_kzalloc(ctx->dev, size, GFP_KERNEL);
	if (!ctx->cmds)
		return false;

	cmdq = virt_to_phys(ctx->cmds);

	for (i = 0; i < XDMA_NUM_CMDS; ++i) {
		struct aspeed_xdma_cmd *cmd = &ctx->cmds[i];

		memset(cmd, 0, size);
		cmd->comm1_hi = FIELD_PREP(XDMA_CMD_COMM1_HI_HOST_PITCH, 1) |
				FIELD_PREP(XDMA_CMD_COMM1_HI_BMC_PITCH, 1);
	}

	dev_info(ctx->dev, "cmd queue at %08x\n", (u32)cmdq);

	aspeed_xdma_write(ctx, XDMA_BMC_CMD_QUEUE_ADDR, cmdq);
	aspeed_xdma_write(ctx, XDMA_BMC_CMD_QUEUE_ENDP, XDMA_CMD_SIZE);
	aspeed_xdma_write(ctx, XDMA_BMC_CMD_QUEUE_WRITEP, 0);
	aspeed_xdma_write(ctx, XDMA_CTRL, ctrl);

	return true;
}

#ifdef PROTOTYPE
static ssize_t aspeed_xdma_show_bmc_addr(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%08x\n", ctx->op.bmc_addr);
}

static ssize_t aspeed_xdma_store_bmc_addr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int rc;
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	rc = kstrtou32(buf, 0, &ctx->op.bmc_addr);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(bmc_addr, 0644, aspeed_xdma_show_bmc_addr,
		   aspeed_xdma_store_bmc_addr);

static ssize_t aspeed_xdma_show_host_addr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%016llx\n", ctx->op.host_addr);
}

static ssize_t aspeed_xdma_store_host_addr(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int rc;
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	rc = kstrtou64(buf, 0, &ctx->op.host_addr);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(host_addr, 0644, aspeed_xdma_show_host_addr,
		   aspeed_xdma_store_host_addr);

static ssize_t aspeed_xdma_show_len(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%08x\n", ctx->op.len);
}

static ssize_t aspeed_xdma_store_len(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	rc = kstrtou32(buf, 0, &ctx->op.len);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(len, 0644, aspeed_xdma_show_len, aspeed_xdma_store_len);

static ssize_t aspeed_xdma_execute(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc;
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	if (!ctx->op.bmc_addr || !ctx->op.host_addr || !ctx->op.len || !count)
		return -EINVAL;

	if (test_bit(XDMA_IN_PRG, &ctx->flags))
		return -EBUSY;

	if (buf[0] == 'u')
		ctx->op.upstream = true;
	else
		ctx->op.upstream = false;

	aspeed_xdma_start(ctx, &ctx->op);

	rc = wait_event_interruptible(ctx->wait,
				      !test_bit(XDMA_IN_PRG, &ctx->flags));
	if (rc)
		return -EINTR;

	return count;
}
static DEVICE_ATTR(execute, 0200, NULL, aspeed_xdma_execute);

#define misc2xdma(x) container_of((x), struct aspeed_xdma, misc)

static loff_t aspeed_xdma_vga_seek(struct file *file, loff_t offset,
				   int whence)
{
	switch (whence) {
	case SEEK_CUR:
		break;
	case SEEK_SET:
		file->f_pos = offset;
		break;
	default:
		return -EINVAL;
	}

	return offset;
}

static ssize_t aspeed_xdma_vga_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	int rc;
	void *tbuf;
	struct miscdevice *misc = file->private_data;
	struct aspeed_xdma *ctx = misc2xdma(misc);
	loff_t offs = *offset;

	if (len + offs > ctx->vga_size)
		return -EINVAL;

	tbuf = kmalloc(len, GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	memcpy_fromio(tbuf, ctx->vga + offs, len);

	rc = copy_to_user(buf, tbuf, len);
	if (rc) {
		kfree(tbuf);
		return rc;
	}

	*offset += len;

	kfree(tbuf);
	return len;
}

static ssize_t aspeed_xdma_vga_write(struct file *file, const char __user *buf,
				     size_t len, loff_t *offset)
{
	int rc;
	void *tbuf;
	struct miscdevice *misc = file->private_data;
	struct aspeed_xdma *ctx = misc2xdma(misc);
	loff_t offs = *offset;

	if (len + offs > ctx->vga_size)
		return -EINVAL;

	tbuf = kmalloc(len, GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	rc = copy_from_user(tbuf, buf, len);
	if (rc) {
		kfree(tbuf);
		return rc;
	}

	memcpy_toio(ctx->vga + offs, tbuf, len);

	*offset += len;

	kfree(tbuf);
	return len;
}

static const struct file_operations aspeed_xdma_fops = {
	.owner	= THIS_MODULE,
	.llseek	= aspeed_xdma_vga_seek,
	.read	= aspeed_xdma_vga_read,
	.write	= aspeed_xdma_vga_write,
};
#endif /* PROTOTYPE */

static int aspeed_xdma_probe(struct platform_device *pdev)
{
	int irq;
	int rc;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct aspeed_xdma *ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctx);
	ctx->dev = dev;
	init_waitqueue_head(&ctx->wait);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->base)) {
		dev_err(dev, "Unable to ioremap registers\n");
		return PTR_ERR(ctx->base);
	}

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq) {
		dev_err(dev, "Unable to find IRQ\n");
		return -ENODEV;
	}

	rc = devm_request_irq(dev, irq, aspeed_xdma_irq, IRQF_SHARED,
			      DEVICE_NAME, ctx);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	ctx->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(ctx->reset)) {
		dev_err(dev, "Unable to request reset control\n");
		return PTR_ERR(ctx->reset);
	}

	if (!aspeed_xdma_init(ctx)) {
		dev_err(dev, "Failed to allocate memory for cmds\n");
		return -ENOMEM;
	}

#ifdef PROTOTYPE
{
	u32 mem_size = 0x20000000;
	u32 scu_conf;
	u32 v;
	u32 vga;
	const u32 conf = SCU_PCIE_CONF_BMC_EN | SCU_PCIE_CONF_BMC_EN_IRQ |
		SCU_PCIE_CONF_BMC_EN_DMA;
	struct regmap *scu =
		syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
	const u32 mem_sizes[4] = { 0x8000000, 0x10000000, 0x20000000,
				   0x40000000 };
	const u32 vga_sizes[4] = { 0x800000, 0x1000000, 0x2000000, 0x4000000 };

	if (IS_ERR(scu)) {
		dev_err(dev, "Unable to grab SCU regs\n");
		return PTR_ERR(scu);
	}

	regmap_read(scu, SCU_PCIE_CONF, &v);
	dev_info(dev, "old pcie conf %08x\n", v);
	regmap_update_bits(scu, SCU_PCIE_CONF, conf | SCU_PCIE_CONF_VGA_EN,
			   conf);
	regmap_read(scu, SCU_PCIE_CONF, &v);
	dev_info(dev, "new pcie conf %08x\n", v);

	regmap_read(scu, SCU_STRAP, &scu_conf);
	ctx->vga_size = vga_sizes[FIELD_GET(SCU_STRAP_VGA_MEM, scu_conf)];

	{
		void __iomem *sdmc_base = ioremap(0x1e6e0000, 0x100);

		if (!IS_ERR(sdmc_base)) {
			u32 sdmc = readl(sdmc_base + SDMC_CONF);

			mem_size = mem_sizes[sdmc & SDMC_CONF_MEM];
			iounmap(sdmc_base);
		}
	}

	vga = (mem_size - ctx->vga_size) + 0x80000000;
	dev_info(dev, "vga mem size %08x at %08x\n", ctx->vga_size, vga);

	ctx->vga = ioremap(vga, ctx->vga_size);
	if (IS_ERR(ctx->vga)) {
		dev_err(dev, "Unable to remap vga mem\n");
		return PTR_ERR(ctx->vga);
	}
/*
	rc = of_reserved_mem_device_init(dev);
	if (rc) {
		dev_err(dev, "Unable to reserve memory\n");
		return rc;
	}

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "Failed to set DMA mask\n");
		of_reserved_mem_device_release(dev);
		return rc;
	}

	ctx->vga.virt = dma_alloc_coherent(ctx->dev, ctx->vga_size,
					   &ctx->vga.dma, GFP_KERNEL);
	if (!ctx->vga.virt) {
		dev_err(dev, "Failed to allocate VGA mem\n");
		of_reserved_mem_device_release(dev);
		return -ENOMEM;
	}

	dev_info(dev, "alloc'd vga at %08x\n", (u32)ctx->vga.dma);
*/

	device_create_file(dev, &dev_attr_bmc_addr);
	device_create_file(dev, &dev_attr_host_addr);
	device_create_file(dev, &dev_attr_len);
	device_create_file(dev, &dev_attr_execute);

	ctx->misc.minor = MISC_DYNAMIC_MINOR;
	ctx->misc.fops = &aspeed_xdma_fops;
	ctx->misc.name = "xdma-vga-mem";
	ctx->misc.parent = dev;
	rc = misc_register(&ctx->misc);
	if (rc) {
		dev_err(dev, "Unable to register vgamem miscdevice\n");
		of_reserved_mem_device_release(dev);
		return rc;
	}
}
#endif /* PROTOTYPE */

	reset_control_deassert(ctx->reset);

	return 0;
}

static int aspeed_xdma_remove(struct platform_device *pdev)
{
	struct aspeed_xdma *ctx = platform_get_drvdata(pdev);

#ifdef PROTOTYPE
	misc_deregister(&ctx->misc);

	device_remove_file(ctx->dev, &dev_attr_execute);
	device_remove_file(ctx->dev, &dev_attr_len);
	device_remove_file(ctx->dev, &dev_attr_host_addr);
	device_remove_file(ctx->dev, &dev_attr_bmc_addr);
/*
	dma_free_coherent(ctx->dev, ctx->vga_size, ctx->vga.virt,
			  ctx->vga.dma);

	of_reserved_mem_device_release(ctx->dev); */
	iounmap(ctx->vga);
#endif /* PROTOTYPE */

	reset_control_assert(ctx->reset);

	return 0;
}

static const struct of_device_id aspeed_xdma_match[] = {
	{ .compatible = "aspeed,ast2500-xdma" },
	{ },
};

static struct platform_driver aspeed_xdma_driver = {
	.probe = aspeed_xdma_probe,
	.remove = aspeed_xdma_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_xdma_match,
	},
};

module_platform_driver(aspeed_xdma_driver);

MODULE_AUTHOR("Eddie James");
MODULE_DESCRIPTION("Aspeed X-DMA Driver");
MODULE_LICENSE("GPLv2");
