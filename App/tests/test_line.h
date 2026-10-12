/**
 * @file test_line.h
 * @brief Fixed-length text line used by the serial bring-up tests.
 *
 * A line is 64 bytes: a two-character tag, an 8-digit hex sequence number, a
 * pattern derived from the sequence, a checksum and CR LF:
 *
 *   U5 0000002A <47 pattern characters>*5C\r\n
 *
 * It is readable in a terminal and can be verified from any starting point in
 * a byte stream.
 */
#ifndef TEST_LINE_H
#define TEST_LINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TEST_LINE_LENGTH 64U
#define TEST_LINE_TAG_LENGTH 2U

typedef struct
{
  uint8_t data[TEST_LINE_LENGTH];
  size_t fill;
  bool overflowed;
} test_line_reader_t;

typedef struct
{
  bool valid; /* Length, framing and checksum are correct. */
  char tag[TEST_LINE_TAG_LENGTH];
  uint32_t sequence;
} test_line_t;

/** Write one line into @p buffer, which must hold TEST_LINE_LENGTH bytes. */
void test_line_build(uint8_t *buffer, const char *tag, uint32_t sequence);

/**
 * Feed one received byte to the reader.
 *
 * @return true when the byte ended a line; @p line then describes it.
 */
bool test_line_feed(test_line_reader_t *reader, uint8_t byte, test_line_t *line);

/** Whether a line carries the given two-character tag. */
bool test_line_has_tag(const test_line_t *line, const char *tag);

#endif /* TEST_LINE_H */
