uint16_t term_columns = 0, term_rows = 0;

void TermGetCaps(void) {
    char report[32];
    int i;
    long startmillis = millis();
    memset(report, 0, sizeof(report));

    // Goto far enough and request cursor position report
    Serial.println("\033[2J\033H\033[1000;1000H\033[6n");

    for (int i = 0; i < sizeof(report); i++) {
        while (!Serial.available() && millis() - startmillis < 250);
        report[i] = Serial.read();
        if (report[i] == 'R') break;
    }    
    report[sizeof(report) - 1] = 0;
        
    if (report[0] != '\033') {
        // some sad terminal without capabilities and probably without control sequences
        term_rows = 0;
        term_columns = 0;
        Serial.println("\nTerminal is dumb. Fine.");
        return;
    }
    
    char *word = strtok(report+2, ";");
    term_rows = atoi(word);
    word = strtok(NULL, "R");
    term_columns = atoi(word);
    Serial.print("Terminal rows="); Serial.print(term_rows); 
    Serial.print(" columns="); Serial.println(term_columns);
}

boolean TermCanPositionCursor() {
    return term_columns != 0;
}

void TermHome(void) {
    if (term_columns != 0) Serial.print("  \033[H");
}

void TermClear(void) {
    if (term_columns != 0) Serial.print("\033[2J\033[H");
}

void TermGoto(uint16_t y, uint16_t x)
{
    if (term_columns != 0) {
        Serial.print("\033["); Serial.print(y); Serial.print(";"); Serial.print(x); Serial.print("H");
    }
}
