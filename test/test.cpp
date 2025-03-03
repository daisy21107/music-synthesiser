#include <bitset>

// Assuming DigitalWrite and DigitalRead are predefined functions
void DigitalWrite(int pin, bool state);
bool DigitalRead(int pin);

#define RA0 2  // Example GPIO pin numbers
#define RA1 3
#define RA2 4
#define REN 5
#define C0  6
#define C1  7
#define C2  8
#define C3  9

std::bitset<4> readCols() {
    std::bitset<4> result;

    // Set all row select addresses LOW to select Row 0
    DigitalWrite(RA0, LOW);
    DigitalWrite(RA1, LOW);
    DigitalWrite(RA2, LOW);
    DigitalWrite(REN, HIGH);  // Enable row selection

    // Read the column states
    result[0] = DigitalRead(C0);
    result[1] = DigitalRead(C1);
    result[2] = DigitalRead(C2);
    result[3] = DigitalRead(C3);

    return result;
}
