// SPDX-License-Identifier: GPL-2.0

#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#define DEVICE_NAME			"aspeed-xdma"

#define XDMA_MAX_BYTES			0x800000 /* 8MB VGA memory space */
#define XDMA_NUM_DESCS			32

#define XDMA_CTRL			0x20
#define  XDMA_CTRL_US_COMP		BIT(4)
#define  XDMA_CTRL_DS_COMP		BIT(5)
#define  XDMA_CTRL_DS_DIRTY		BIT(6)

struct aspeed_xdma_desc_cmd {
	u32 cmd0_low;
	u32 cmd0_high;
	u32 cmd1_low;
	u32 cmd1_high;
	u32 cmd2_low;
	u32 cmd2_high;
	u32 resv_low;
	u32 resv_high;
};

struct aspeed_xdma_desc {
	struct dma_async_tx_descriptor desc;
	struct aspeed_xdma_desc_cmd cmd;

	struct list_head chan_queue;
	struct list_head tx_list;
};

struct aspeed_xdma {
	struct dma_chan chan;
	struct dma_device dma;
	void __iomem *base;

	struct aspeed_xdma_desc *descs;
	struct list_head free_descs;
	struct list_head pending_descs;
	spinlock_t list_lock;

	struct reset_control *reset;
};

#define chan2aspeed(x) container_of((x), struct aspeed_xdma, chan)

static u32 aspeed_xdma_read(struct aspeed_xdma *ctx, u32 reg)
{
	u32 v = readl(ctx->base + reg);

	dev_dbg(ctx->dma.dev, "read %02x[%08x]\n", reg, v);
	return v;
}

static void aspeed_xdma_write(struct aspeed_xdma *ctx, u32 reg, u32 val)
{
	writel(val, ctx->base + reg);
	dev_dbg(ctx->dma.dev, "write %02x[%08x]\n", reg,
		readl(ctx->base + reg));
}

static void aspeed_xdma_start()
{

}

static dma_cookie_t aspeed_xdma_tx_submit(struct dma_async_tx_descriptor *tx)
{

}

static int aspeed_xdma_alloc_chan_resources(struct dma_chan *chan)
{
	int i;
	int rc;
	struct device *dev = chan2dmadev(chan);
	struct aspeed_xdma *ctx = chan2aspeed(chan);
	struct aspeed_xdma_desc *ax_desc;

	ctx->descs = kcalloc(XDMA_NUM_DESCS, sizeof(*desc), GFP_KERNEL);

	if (!ctx->descs)
		return -ENOMEM;

	for (i = 0; i < XDMA_NUM_DESCS; ++i) {
		ax_desc = ctx->descs[i];

		dma_async_tx_descriptor_init(&ax_desc->desc, chan);
		ax_desc->desc.tx_submit = aspeed_xdma_tx_submit;

		list_add_tail(&ax_desc->chan_queue, &ctx->free_descs);
	}

	dma_cookie_init(chan);
	
/*	
	desc->desc.phys = dma_map_single(dev, &desc->cmd,
					 sizeof(struct aspeed_xdma_desc_cmd),
					 DMA_TO_DEVICE);

	rc = dma_mapping_error(dev, desc->desc.phys);
	if (rc) {
		dev_err(dev, "DMA mapping error for desc cmd: %d\n", rc);
		return rc;
	}

	dma_cookie_init(chan);
*/

	return XDMA_NUM_DESCS;
}

static void aspeed_xdma_free_chan_resources(struct dma_chan *chan)
{
	struct aspeed_xdma *ctx = chan2aspeed(chan);

	kfree(ctx->descs);
}

static struct aspeed_xdma_desc *aspeed_xdma_get_desc(struct aspeed_xdma *ctx)
{
	unsigned long flags;
	struct aspeed_xdma_desc *ax_desc;
	struct aspeed_xdma_desc *ret = NULL;
	struct aspeed_xdma_desc *tmp;

	spin_lock_irqsave(&ctx->list_lock, flags);
	list_for_each_entry_safe(ax_desc, tmp, &ctx->free_descs, chan_queue) {
		if (async_tx_test_ack(&ax_desc->desc)) {
			list_del(&ax_desc->chan_queue);
			ret = ax_desc;
			break;
		}
	}
	spin_unlock_irqrestore(&ctx->list_lock, flags);

	return ret;
}

static void aspeed_xdma_prep_desc(struct aspeed_xdma_desc *ax_desc,
				  struct scatterlist *sg,
				  struct dma_transfer_direction dir)
{
	if (dma == DMA_MEM_TO_DEV) {
		// from bmc to host
	} else {
		// from host to bmc
	}
}

static struct dma_async_tx_descriptor *aspeed_xdma_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct scatterlist *sg;
	struct device *dev = chan2dmadev(chan);
	struct aspeed_xdma *ctx = chan2aspeed(chan);

	for_each_sg(sgl, sg, sg_len, i) {
		struct aspeed_xdma_desc *ax_desc = aspeed_xdma_get_desc(ctx);

		if (!ax_desc)
			goto err;

		
		// fetch descriptor
		// write descriptor values
		// add it to tx_list
		size_t len = sg_dma_len(sg);

		if (len > XDMA_MAX_BYTES) {
			goto fail;
		}

		desc = ep93xx_dma_desc_get(edmac);
		if (!desc) {
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = sg_dma_address(sg);
			desc->dst_addr = edmac->runtime_addr;
		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = sg_dma_address(sg);
		}

		desc->size = len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

}

static irqreturn_t aspeed_xdma_irq(int irq, void *arg)
{
	struct aspeed_xdma *ctx = arg;
}

static int aspeed_xdma_probe(struct platform_device *pdev)
{
	int i;
	int irq;
	int rc;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct dma_chan *chan;
	struct dma_device *dma;
	struct aspeed_xdma *ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

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

	INIT_LIST_HEAD(&ctx->free_descs);
	INIT_LIST_HEAD(&ctx->pending_descs);

	dma = &ctx->dma;
	INIT_LIST_HEAD(&dma->channels);
	dma_cap_set(DMA_SLAVE, dma->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma->src_addr_widths = DMA_SLAVE_BUSWIDTH_8_BYTES;
	dma->dst_addr_widths = DMA_SLAVE_BUSWIDTH_8_BYTES;
	dma->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dma->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma->dev = dev;

	chan = &ctx->chan;
	chan->device = dma;
	chan->chan_id = 0;
	list_add_tail(&chan->device_node, &dma->channels);

	rc = dma_async_device_register(dma);
	if (rc) {
		dev_err(dev, "Unable to register dma device\n");
		return rc;
	}

	reset_control_deassert(ctx->reset);

	return 0;
}

static const struct of_device_id aspeed_xdma_match[] = {
	{ .compatible = "aspeed,aspeed-xdma" },
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
