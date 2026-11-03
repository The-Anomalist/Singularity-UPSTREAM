/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_SINGULARITY_AUDIO_H
#define __SOC_SINGULARITY_AUDIO_H

#if IS_ENABLED(CONFIG_SND_SOC_SINGULARITY_AUDIO)
int snd_soc_singularity_audio_add(struct snd_soc_card *card);
#else
static inline int snd_soc_singularity_audio_add(struct snd_soc_card *card)
{
	return 0;
}
#endif

#endif
