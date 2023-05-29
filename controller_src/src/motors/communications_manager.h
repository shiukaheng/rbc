// Class for parsing input / sending output with serial
// Messages are separated by newline
// Messages are in the form of:
//     <command> <arg1> <arg2> ... <checksum>

// Class statically declares callback function for each command (but makes it easily declarable)
// Supported data types: int, float, double, bool, string

// This will replace the SerialPIDClient / SerialPIDClient2 classes with a architecture that is more encapsulated
// Also, instead of communicating float values through strings, we will use a binary protocol which is more efficient