#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <cstring>
#include <sstream>
#include "pocsag.h"
#include <iostream>
//Check out main() at the bottom of the file
//You can modify MIN_DELAY and MAX_DELAY to fit your needs.


//Check out https://en.wikipedia.org/wiki/POCSAG
//Also see http://www.itu.int/dms_pubrec/itu-r/rec/m/R-REC-M.584-2-199711-I!!PDF-E.pdf
//They'll be handy when trying to understand this stuff.

//The sync word exists at the start of every batch.
//A batch is 16 words, so a sync word occurs every 16 data words.
#define SYNC 0x7CD215D8

//The idle word is used as padding before the address word, and at the end
//of a message to indicate that the message is finished. Interestingly, the
//idle word does not have a valid CRC code, while the sync word does.
#define IDLE 0x7A89C197

//One frame consists of a pair of two words
#define FRAME_SIZE 2

//One batch consists of 8 frames, or 16 words
#define BATCH_SIZE 16

//The preamble comes before a message, and is a series of alternating
//1,0,1,0... bits, for at least 576 bits. It exists to allow the receiver
//to synchronize with the transmitter
#define PREAMBLE_LENGTH 576


//These bits appear as the first bit of a word, 0 for an address word and
//one for a data word
#define FLAG_ADDRESS 0x000000
#define FLAG_MESSAGE 0x100000


//The last two bits of an address word's data represent the data type
//0x3 for text, and 0x0 for numeric.
#define FLAG_TEXT_DATA 0x3
#define FLAG_NUMERIC_DATA = 0x0;

//Each data word can contain 20 bits of text information. Each character is
//7 bits wide, ASCII encoded. The bit order of the characters is reversed from
//the normal bit order; the most significant bit of a word corresponds to the
//least significant bit of a character it is encoding. The characters are split
//across the words of a message to ensure maximal usage of all bits.
#define TEXT_BITS_PER_WORD 20

//As mentioned above, characters are 7 bit ASCII encoded
#define TEXT_BITS_PER_CHAR 7

//Length of CRC codes in bits
#define CRC_BITS 10

//The CRC generator polynomial
#define CRC_GENERATOR 0b11101101001

/**
 * Calculate the CRC error checking code for the given word.
 * Messages use a 10 bit CRC computed from the 21 data bits.
 * This is calculated through a binary polynomial long division, returning
 * the remainder.
 * See https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Computation
 * for more information.
 */
uint32_t crc(uint32_t inputMsg) {
    //Align MSB of denominatorerator with MSB of message
    uint32_t denominator = CRC_GENERATOR << 20;

    //Message is right-padded with zeroes to the message length + crc length
    uint32_t msg = inputMsg << CRC_BITS;

    //We iterate until denominator has been right-shifted back to it's original value.
    for (int column = 0; column <= 20; column++) {
        //Bit for the column we're aligned to
        int msgBit = (msg >> (30 - column)) & 1;

        //If the current bit is zero, we don't modify the message this iteration
        if (msgBit != 0) {
            //While we would normally subtract in long division, we XOR here.
            msg ^= denominator;
        }

        //Shift the denominator over to align with the next column
        denominator >>= 1;
    }

    //At this point 'msg' contains the CRC value we've calculated
    return msg & 0x3FF;
}


/**
 * Calculates the even parity bit for a message.
 * If the number of bits in the message is even, return 0, else return 1.
 */
uint32_t parity(uint32_t x) {
    //Our parity bit
    uint32_t p = 0;

    //We xor p with each bit of the input value. This works because
    //xoring two one-bits will cancel out and leave a zero bit.  Thus
    //xoring any even number of one bits will result in zero, and xoring
    //any odd number of one bits will result in one.
    for (int i = 0; i < 32; i++) {
        p ^= (x & 1);
        x >>= 1;
    }
    return p;
}

/**
 * Encodes a 21-bit message by calculating and adding a CRC code and parity bit.
 */
uint32_t encodeCodeword(uint32_t msg) {
    uint32_t fullCRC = (msg << CRC_BITS) | crc(msg);
    uint32_t p = parity(fullCRC);
    return (fullCRC << 1) | p;
}

/**
 * ASCII encode a null-terminated string as a series of codewords, written
 * to (*out). Returns the number of codewords written. Caller should ensure
 * that enough memory is allocated in (*out) to contain the message
 *
 * initial_offset indicates which word in the current batch the function is
 * beginning at, so that it can insert SYNC words at appropriate locations.
 */
uint32_t encodeASCII(uint32_t initial_offset, char* str, uint32_t* out) {
    //Number of words written to *out
    uint32_t numWordsWritten = 0;

    //Data for the current word we're writing
    uint32_t currentWord = 0;

    //Nnumber of bits we've written so far to the current word
    uint32_t currentNumBits = 0;

    //Position of current word in the current batch
    uint32_t wordPosition = initial_offset;

    while (*str != 0) {
        unsigned char c = *str;
        str++;
        //Encode the character bits backwards
        for (int i = 0; i < TEXT_BITS_PER_CHAR; i++) {
            currentWord <<= 1;
            currentWord |= (c >> i) & 1;
            currentNumBits++;
            if (currentNumBits == TEXT_BITS_PER_WORD) {
                //Add the MESSAGE flag to our current word and encode it.
                *out = encodeCodeword(currentWord | FLAG_MESSAGE);
                out++;
                currentWord = 0;
                currentNumBits = 0;
                numWordsWritten++;

                wordPosition++;
                if (wordPosition == BATCH_SIZE) {
                    //We've filled a full batch, time to insert a SYNC word
                    //and start a new one.
                    *out = SYNC;
                    out++;
                    numWordsWritten++;
                    wordPosition = 0;
                }
            }
        }
    }

    //Write remainder of message
    if (currentNumBits > 0) {
        //Pad out the word to 20 bits with zeroes
        currentWord <<= 20 - currentNumBits;
        *out = encodeCodeword(currentWord | FLAG_MESSAGE);
        out++;
        numWordsWritten++;

        wordPosition++;
        if (wordPosition == BATCH_SIZE) {
            //We've filled a full batch, time to insert a SYNC word
            //and start a new one.
            *out = SYNC;
            out++;
            numWordsWritten++;
            wordPosition = 0;
        }
    }

    return numWordsWritten;
}

/**
 * An address of 21 bits, but only 18 of those bits are encoded in the address
 * word itself. The remaining 3 bits are derived from which frame in the batch
 * is the address word. This calculates the number of words (not frames!)
 * which must precede the address word so that it is in the right spot. These
 * words will be filled with the idle value.
 */
int addressOffset(int address) {
    return (address & 0x3) * FRAME_SIZE;
}

/**
 * Encode a full text POCSAG transmission addressed to (address).
 * (*message) is a null terminated C string.
 * (*out) is the destination to which the transmission will be written.
 */
void encodeTransmission(int address, char* message, uint32_t* out) {

    
    //Encode preamble
    //Alternating 1,0,1,0 bits for 576 bits, used for receiver to synchronize
    //with transmitter
    for (int i = 0; i < PREAMBLE_LENGTH / 32; i++) {
        *out = 0xAAAAAAAA;
        out++;
    }

    uint32_t* start = out;

    //Sync
    *out = SYNC;
    out++;

    //Write out padding before address word
    int prefixLength = addressOffset(address);
    for (int i = 0; i < prefixLength; i++) {
        *out = IDLE;
        out++;
    }

    //Write address word.
    //The last two bits of word's data contain the message type
    //The 3 least significant bits are dropped, as those are encoded by the
    //word's location.
    *out = encodeCodeword( ((address >> 3) << 2) | FLAG_TEXT_DATA);
    out++;

    //Encode the message itself
    out += encodeASCII(addressOffset(address) + 1, message, out);


    //Finally, write an IDLE word indicating the end of the message
    *out = IDLE;
    out++;
    
    //Pad out the last batch with IDLE to write multiple of BATCH_SIZE + 1
    //words (+ 1 is there because of the SYNC words)
    size_t written = out - start;
    size_t padding = (BATCH_SIZE + 1) - written % (BATCH_SIZE + 1);
    for (int i = 0; i < padding; i++) {
        *out = IDLE;
        out++;
    }
}

/**
 * Calculates the length in words of a text POCSAG message, given the address
 * and the number of characters to be transmitted.
 */
size_t textMessageLength(int address, int numChars) {
    size_t numWords = 0;

    //Padding before address word.
    numWords += addressOffset(address);

    //Address word itself
    numWords++;

    //numChars * 7 bits per character / 20 bits per word, rounding up
    numWords += (numChars * TEXT_BITS_PER_CHAR + (TEXT_BITS_PER_WORD - 1))
                    / TEXT_BITS_PER_WORD;

    //Idle word representing end of message
    numWords++;

    //Pad out last batch with idles
    numWords += BATCH_SIZE - (numWords % BATCH_SIZE);

    //Batches consist of 16 words each and are preceded by a sync word.
    //So we add one word for every 16 message words
    numWords += numWords / BATCH_SIZE;

    //Preamble of 576 alternating 1,0,1,0 bits before the message
    //Even though this comes first, we add it to the length last so it
    //doesn't affect the other word-based calculations
    numWords += PREAMBLE_LENGTH / 32;

    return numWords;
}


size_t getColonIndex(char* line) {
    // Find the colon separating the address from the message
    // returns -1 if no colon is found
    size_t colonIndex = 0;
    for (size_t i = 0; i < sizeof(line); i++) {
        if (line[i] == 0) {
            return -1;
        }
        if (line[i] == ':') {
            colonIndex = i;
            return colonIndex;
        }
    }
    return -1;
}

size_t encodeString(std::string input, uint32_t* transmission) {
    int address = 0;
    size_t transmissionLength = textMessageLength(address, input.length());
    encodeTransmission(address, (char*)input.c_str(), transmission);
    return transmissionLength;
}

std::string decodePOCSAG(uint32_t* transmission, size_t transmissionLength) {
    std::string message = "";
    for (int i = 0; i < transmissionLength; i++) {
        uint32_t codeWord = transmission[i];
        if (codeWord == 0xAAAAAAAA) {
            //preamble
            continue;
        }
        if (codeWord == 0x7CD215D8) {
            //sync
            std::cout << "found sync" << std::endl;
            continue;
        }
        if (codeWord == 0x7A89C197) {
            //idle
            std::cout << "found idle" << std::endl;
            break;
        }
        std::cout << codeWord << std::endl;
        uint32_t data = codeWord >> 2;
        uint32_t type = data & 0x3;
        if (type == 0x3) {
            //text data
            for (int j = 0; j < 10; j++) {
                uint32_t character = data >> (20 - 7 * j);
                character &= 0x7F;
                message += (char)character;
            }
        }
    }
    return message;
}
