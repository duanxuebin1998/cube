#include "display_language.h"
#include <stddef.h>

bool DisplayLanguage_IsValid(int32_t raw_language)
{
    return (raw_language >= (int32_t)LANGUAGE_CHINESE) &&
           (raw_language < (int32_t)LANGUAGE_COUNT);
}

LANGUAGE_TYPE DisplayLanguage_Normalize(int32_t raw_language)
{
    if (!DisplayLanguage_IsValid(raw_language)) {
        return LANGUAGE_ENGLISH;
    }

    return (LANGUAGE_TYPE)raw_language;
}

LANGUAGE_TYPE DisplayLanguage_Resolve(int32_t raw_language, DisplayTextDomain domain)
{
    LANGUAGE_TYPE language = DisplayLanguage_Normalize(raw_language);

    if ((domain == DISPLAY_TEXT_DOMAIN_LEGACY) &&
        (language != LANGUAGE_CHINESE)) {
        return LANGUAGE_ENGLISH;
    }

    return language;
}

uint8_t DisplayLanguage_GetLegacyColumn(int32_t raw_language)
{
    return (DisplayLanguage_Resolve(raw_language, DISPLAY_TEXT_DOMAIN_LEGACY) ==
            LANGUAGE_CHINESE) ? 0U : 1U;
}

bool DisplayLanguage_IsRussian(int32_t raw_language)
{
    return DisplayLanguage_Normalize(raw_language) == LANGUAGE_RUSSIAN;
}

const char *DisplayLanguage_SelectLegacyText(int32_t raw_language,
                                             const char *chinese,
                                             const char *english)
{
    if ((DisplayLanguage_GetLegacyColumn(raw_language) == 1U) &&
        (english != NULL)) {
        return english;
    }

    return chinese;
}

const char *DisplayLanguage_SelectStatusText(int32_t raw_language,
                                             const char *chinese,
                                             const char *english,
                                             const char *russian)
{
    LANGUAGE_TYPE language = DisplayLanguage_Resolve(raw_language,
                                                     DISPLAY_TEXT_DOMAIN_STATUS);

    if ((language == LANGUAGE_RUSSIAN) && (russian != NULL)) {
        return russian;
    }
    if ((language != LANGUAGE_CHINESE) && (english != NULL)) {
        return english;
    }

    return chinese;
}
