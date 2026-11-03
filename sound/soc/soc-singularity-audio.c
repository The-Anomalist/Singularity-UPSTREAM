// SPDX-License-Identifier: GPL-2.0
/* Singularity: root-writable aliases for existing, scalar ALSA gain controls. */
#include <linux/device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "soc-singularity-audio.h"

struct singularity_gain_attr {
	struct device_attribute dev_attr;
	struct snd_ctl_elem_id id;
	long min;
	long max;
	long step;
};

struct singularity_text_attr {
	struct device_attribute dev_attr;
	char *text;
};

/* Exact names, optionally preceded by an ASoC component's name prefix. */
static const char * const singularity_gain_names[] = {
	"HPHL Volume", "HPHR Volume",
	"ADC1 Volume", "ADC2 Volume", "ADC3 Volume", "ADC4 Volume",
	"RX_RX0 Digital Volume", "RX_RX1 Digital Volume",
	"RX_RX2 Digital Volume",
	"WSA_RX0 Digital Volume", "WSA_RX1 Digital Volume",
	"TX_DEC0 Volume", "TX_DEC1 Volume", "TX_DEC2 Volume",
	"TX_DEC3 Volume", "TX_DEC4 Volume", "TX_DEC5 Volume",
	"TX_DEC6 Volume", "TX_DEC7 Volume",
};

static bool singularity_gain_allowed(const char *name)
{
	size_t len = strlen(name);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(singularity_gain_names); i++) {
		const char *suffix = singularity_gain_names[i];
		size_t n = strlen(suffix);

		if (len >= n && !strcmp(name + len - n, suffix) &&
		    (len == n || name[len - n - 1] == ' '))
			return true;
	}
	return false;
}

static int singularity_gain_lookup(struct snd_card *card,
		struct singularity_gain_attr *gain, struct snd_kcontrol **result)
{
	struct snd_kcontrol *ctl;
	struct snd_ctl_elem_info info;
	unsigned int access;
	int ret;

	if (card->shutdown)
		return -ENODEV;
	ctl = snd_ctl_find_id(card, &gain->id);
	if (!ctl || strcmp(ctl->id.name, gain->id.name) ||
	    ctl->id.index != gain->id.index ||
	    ctl->id.iface != gain->id.iface ||
	    ctl->id.device != gain->id.device ||
	    ctl->id.subdevice != gain->id.subdevice || ctl->count != 1)
		return -ENOENT;
	access = ctl->vd[0].access;
	if ((access & SNDRV_CTL_ELEM_ACCESS_INACTIVE) || !ctl->info ||
	    !ctl->get || !ctl->put ||
	    !(access & SNDRV_CTL_ELEM_ACCESS_READ) ||
	    !(access & SNDRV_CTL_ELEM_ACCESS_WRITE))
		return -EACCES;
	memset(&info, 0, sizeof(info));
	info.id = gain->id;
	ret = ctl->info(ctl, &info);
	if (ret < 0)
		return ret;
	if (info.type != SNDRV_CTL_ELEM_TYPE_INTEGER || info.count != 1 ||
	    info.value.integer.min != gain->min ||
	    info.value.integer.max != gain->max ||
	    info.value.integer.step != gain->step)
		return -ESTALE;
	*result = ctl;
	return 0;
}

static ssize_t singularity_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct singularity_gain_attr *gain = container_of(attr,
			struct singularity_gain_attr, dev_attr);
	struct snd_card *card = dev_to_snd_card(dev);
	struct snd_ctl_elem_value *value;
	struct snd_kcontrol *ctl;
	int ret;

	value = kzalloc(sizeof(*value), GFP_KERNEL);
	if (!value)
		return -ENOMEM;
	ret = snd_power_wait(card, SNDRV_CTL_POWER_D0);
	if (ret < 0)
		goto out;
	down_read(&card->controls_rwsem);
	ret = singularity_gain_lookup(card, gain, &ctl);
	if (!ret) {
		value->id = gain->id;
		ret = ctl->get(ctl, value);
		if (ret >= 0)
			ret = scnprintf(buf, PAGE_SIZE, "%ld\n",
					value->value.integer.value[0]);
	}
	up_read(&card->controls_rwsem);
out:
	kfree(value);
	return ret;
}

static ssize_t singularity_gain_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct singularity_gain_attr *gain = container_of(attr,
			struct singularity_gain_attr, dev_attr);
	struct snd_card *card = dev_to_snd_card(dev);
	struct snd_ctl_elem_value *value;
	struct snd_kcontrol *ctl;
	long requested;
	int ret;

	ret = kstrtol(buf, 10, &requested);
	if (ret)
		return ret;
	if (requested < gain->min || requested > gain->max ||
	    (gain->step > 1 && (requested - gain->min) % gain->step))
		return -ERANGE;
	value = kzalloc(sizeof(*value), GFP_KERNEL);
	if (!value)
		return -ENOMEM;
	ret = snd_power_wait(card, SNDRV_CTL_POWER_D0);
	if (ret < 0)
		goto out;
	/* Match ALSA's write-side serialization; respect userspace ownership. */
	down_write(&card->controls_rwsem);
	ret = singularity_gain_lookup(card, gain, &ctl);
	if (!ret && ctl->vd[0].owner)
		ret = -EBUSY;
	if (!ret) {
		value->id = gain->id;
		value->value.integer.value[0] = requested;
		ret = ctl->put(ctl, value);
		if (ret > 0)
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE, &gain->id);
	}
	up_write(&card->controls_rwsem);
out:
	kfree(value);
	return ret < 0 ? ret : count;
}

static ssize_t singularity_text_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct singularity_text_attr *text = container_of(attr,
			struct singularity_text_attr, dev_attr);

	return scnprintf(buf, PAGE_SIZE, "%s", text->text);
}

static void singularity_text_init(struct singularity_text_attr *text,
		const char *name)
{
	sysfs_attr_init(&text->dev_attr.attr);
	text->dev_attr.attr.name = name;
	text->dev_attr.attr.mode = 0444;
	text->dev_attr.show = singularity_text_show;
}

static bool singularity_is_codec(struct snd_soc_card *card,
		struct snd_soc_component *component)
{
	struct snd_soc_pcm_runtime *rtd;
	unsigned int i;

	list_for_each_entry(rtd, &card->rtd_list, list)
		for (i = 0; i < rtd->num_codecs; i++)
			if (rtd->codec_dais[i]->component == component)
				return true;
	return false;
}

int snd_soc_singularity_audio_add(struct snd_soc_card *soc_card)
{
	struct snd_card *card = soc_card->snd_card;
	struct device *dev = soc_card->dev;
	struct singularity_text_attr *controls, *codecs;
	struct snd_soc_component *component;
	struct snd_ctl_elem_info *info;
	struct attribute_group *group;
	struct attribute **attrs;
	struct snd_kcontrol *ctl;
	unsigned int capacity = 0, used = 0;
	size_t controls_len = 0, codecs_len = 0;
	int ret = -ENOMEM;

	/* Called before snd_card_register(), after all components have probed. */
	down_read(&card->controls_rwsem);
	list_for_each_entry(ctl, &card->controls, list)
		if (singularity_gain_allowed(ctl->id.name))
			capacity++;
	group = devm_kzalloc(dev, sizeof(*group), GFP_KERNEL);
	attrs = devm_kcalloc(dev, capacity + 3, sizeof(*attrs), GFP_KERNEL);
	controls = devm_kzalloc(dev, sizeof(*controls), GFP_KERNEL);
	codecs = devm_kzalloc(dev, sizeof(*codecs), GFP_KERNEL);
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!group || !attrs || !controls || !codecs || !info)
		goto out;
	controls->text = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
	codecs->text = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
	if (!controls->text || !codecs->text)
		goto out;

	list_for_each_entry(ctl, &card->controls, list) {
		struct singularity_gain_attr *gain;
		unsigned int access = ctl->vd[0].access;
		char line[192];
		int n;

		if (!singularity_gain_allowed(ctl->id.name) || ctl->count != 1 ||
		    !ctl->info || !ctl->get || !ctl->put ||
		    !(access & SNDRV_CTL_ELEM_ACCESS_READ) ||
		    !(access & SNDRV_CTL_ELEM_ACCESS_WRITE) ||
		    (access & SNDRV_CTL_ELEM_ACCESS_INACTIVE))
			continue;
		memset(info, 0, sizeof(*info));
		info->id = ctl->id;
		if (ctl->info(ctl, info) < 0 ||
		    info->type != SNDRV_CTL_ELEM_TYPE_INTEGER || info->count != 1 ||
		    info->value.integer.min > info->value.integer.max ||
		    info->value.integer.step < 0)
			continue;
		gain = devm_kzalloc(dev, sizeof(*gain), GFP_KERNEL);
		if (!gain)
			goto out;
		gain->id = ctl->id;
		gain->min = info->value.integer.min;
		gain->max = info->value.integer.max;
		gain->step = info->value.integer.step;
		gain->dev_attr.attr.name = devm_kasprintf(dev, GFP_KERNEL,
				"gain_%u", ctl->id.numid);
		if (!gain->dev_attr.attr.name)
			goto out;
		sysfs_attr_init(&gain->dev_attr.attr);
		gain->dev_attr.attr.mode = 0644;
		gain->dev_attr.show = singularity_gain_show;
		gain->dev_attr.store = singularity_gain_store;
		n = scnprintf(line, sizeof(line), "%s\t%ld\t%ld\t%ld\t%s\n",
			gain->dev_attr.attr.name, gain->min, gain->max,
			gain->step, ctl->id.name);
		if (controls_len + n >= PAGE_SIZE) {
			ret = -E2BIG;
			goto out;
		}
		memcpy(controls->text + controls_len, line, n);
		controls_len += n;
		attrs[used++] = &gain->dev_attr.attr;
	}

	list_for_each_entry(component, &soc_card->component_dev_list, card_list) {
		char line[256];
		int n;

		if (!singularity_is_codec(soc_card, component))
			continue;
		n = scnprintf(line, sizeof(line), "%s\t%s\n", component->name,
			component->driver->name ? component->driver->name : "unknown");
		if (codecs_len + n >= PAGE_SIZE) {
			ret = -E2BIG;
			goto out;
		}
		memcpy(codecs->text + codecs_len, line, n);
		codecs_len += n;
	}

	singularity_text_init(controls, "controls");
	singularity_text_init(codecs, "codecs");
	attrs[used++] = &controls->dev_attr.attr;
	attrs[used++] = &codecs->dev_attr.attr;
	group->name = "singularity_audio";
	group->attrs = attrs;
	ret = snd_card_add_dev_attr(card, group);
out:
	up_read(&card->controls_rwsem);
	kfree(info);
	return ret;
}
