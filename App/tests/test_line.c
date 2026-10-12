/**
 * @file test_line.c
 * @brief Fixed-length text line used by the serial bring-up tests.
 */
#include "test_line.h"

#define SEQUENCE_START 3U
#define SEQUENCE_DIGITS 8U
#define PATTERN_START 12U
#define CHECKSUM_MARK 59U
#define CHECKSUM_START 60U
#define CHECKSUM_DIGITS 2U
#define CR_INDEX 62U
#define LF_INDEX 63U
#define PATTERN_LETTERS 26U

static const char HEX_DIGITS[] = "0123456789ABCDEF";

static uint8_t checksum(const uint8_t *data, size_t length)
{
  uint8_t sum = 0U;

  for (size_t i = 0U; i < length; i++)
  {
    sum = (uint8_t)(sum + data[i]);
  }
  return sum;
}

static bool parse_hex(const uint8_t *digits, size_t count, uint32_t *value)
{
  uint32_t result = 0U;

  for (size_t i = 0U; i < count; i++)
  {
    uint8_t digit = digits[i];
    uint32_t nibble;

    if ((digit >= '0') && (digit <= '9'))
    {
      nibble = (uint32_t)(digit - '0');
    }
    else if ((digit >= 'A') && (digit <= 'F'))
    {
      nibble = (uint32_t)(digit - 'A') + 10U;
    }
    else
    {
      return false;
    }
    result = (result << 4U) | nibble;
  }

  *value = result;
  return true;
}

void test_line_build(uint8_t *buffer, const char *tag, uint32_t sequence)
{
  uint8_t sum;

  buffer[0] = (uint8_t)tag[0];
  buffer[1] = (uint8_t)tag[1];
  buffer[2] = ' ';
  for (size_t i = 0U; i < SEQUENCE_DIGITS; i++)
  {
    uint32_t shift = 4U * (SEQUENCE_DIGITS - 1U - i);

    buffer[SEQUENCE_START + i] = (uint8_t)HEX_DIGITS[(sequence >> shift) & 0xFU];
  }
  buffer[SEQUENCE_START + SEQUENCE_DIGITS] = ' ';
  for (size_t i = PATTERN_START; i < CHECKSUM_MARK; i++)
  {
    buffer[i] = (uint8_t)('A' + ((sequence + i) % PATTERN_LETTERS));
  }
  buffer[CHECKSUM_MARK] = '*';
  sum = checksum(buffer, CHECKSUM_MARK);
  buffer[CHECKSUM_START] = (uint8_t)HEX_DIGITS[sum >> 4U];
  buffer[CHECKSUM_START + 1U] = (uint8_t)HEX_DIGITS[sum & 0xFU];
  buffer[CR_INDEX] = '\r';
  buffer[LF_INDEX] = '\n';
}

static void describe(const test_line_reader_t *reader, test_line_t *line)
{
  const uint8_t *data = reader->data;
  uint32_t received_sum;

  line->valid = false;
  line->tag[0] = '?';
  line->tag[1] = '?';
  line->sequence = 0U;

  if (reader->overflowed || (reader->fill != TEST_LINE_LENGTH) || (data[2] != ' ') ||
      (data[CHECKSUM_MARK] != '*') || (data[CR_INDEX] != '\r'))
  {
    return;
  }
  if (!parse_hex(&data[CHECKSUM_START], CHECKSUM_DIGITS, &received_sum) ||
      (received_sum != checksum(data, CHECKSUM_MARK)))
  {
    return;
  }
  if (!parse_hex(&data[SEQUENCE_START], SEQUENCE_DIGITS, &line->sequence))
  {
    return;
  }

  line->tag[0] = (char)data[0];
  line->tag[1] = (char)data[1];
  line->valid = true;
}

bool test_line_feed(test_line_reader_t *reader, uint8_t byte, test_line_t *line)
{
  if (reader->fill < TEST_LINE_LENGTH)
  {
    reader->data[reader->fill] = byte;
    reader->fill++;
  }
  else
  {
    reader->overflowed = true;
  }

  if (byte != '\n')
  {
    return false;
  }

  describe(reader, line);
  reader->fill = 0U;
  reader->overflowed = false;
  return true;
}

bool test_line_has_tag(const test_line_t *line, const char *tag)
{
  return line->valid && (line->tag[0] == tag[0]) && (line->tag[1] == tag[1]);
}
