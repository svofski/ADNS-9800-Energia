#include <SPI.h>

// the firmeware that is uploaded in the ADNS each time it boots

// Firmware "adns9800_srom_A4.txt" from 
// This firmware is Copyright Avago, please refer to them concerning modifications.
#include "adns9800_srom_A6.h"

#include "adns9800_reg.h"

// this pin connects to SS on ADNS-9800 sensor
#define ADNS_CS SS

uint16_t frameRate;
uint16_t squal;
int lastx = 0, lasty = 0;
int16_t rel_x = 0, rel_y = 0;
int16_t abs_x = 0, abs_y = 0;

#define adns_com(X) digitalWrite(ADNS_CS,LOW);X;digitalWrite(ADNS_CS,HIGH)

void setup() {
    Serial.begin(9600);
    
    TermGetCaps();

    pinMode(ADNS_CS, OUTPUT);
    
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(8);

    Serial.println("ADNS-9800");
  
    if (performStartup() != 0) {
        Serial.println("Abort");
        for(;;);
    }

    delay(2000);
    TermClear();
}

void loop() {
    extern uint16_t term_rows, term_columns;
    
    if (UpdatePointer()) {
        TermHome();
        Serial.print("Rel: ");
        printi(rel_x, 4);
        Serial.print(", ");
        printi(rel_y, 4);
                    
        Serial.print(" SQUAL="); printi(squal, 4);
        Serial.print(" FrameRate="); printi(frameRate, 5);
        Serial.print(" @["); printi(abs_x, 5); Serial.print(","); printi(abs_y, 5); Serial.print("]");
        
        if (TermCanPositionCursor()) {
            TermGoto(lasty, lastx);
            Serial.print("  ");
            lasty = term_rows/2 + abs_y / 400;
            lastx = term_columns/2 + abs_x / 100;
            TermGoto(lasty, lastx);
            Serial.print(":)");
        } else {
            Serial.println();
        }
    }
}

// Configure sensor parameters. Try to squeeze as much as we can from this sensor.
void configureSensor(void)
{
    // Set resolution: 0x01 = 50cpi, minimum, 0x44 = 3400 default, 0x8e = 7100, 0xA4 = 8200 (maximum)
    modifyReg("Configuration_I", REG_Configuration_I, 0xA4);
    modifyReg("Configuration_V", REG_Configuration_V, 0x44);
    
    // Disable rest mode, 0x08 = fixed frame rate, disable AGC
    modifyReg("Configuration_II", REG_Configuration_II, 0x08 + 0x10);
    
    // Default value for Shutter_Max_Bound is 0x4e20, this allows long exposure but limits maximum frame rate.
    // Frame rate, defined by Frame_Period_Max_Bound register, which must be written last in this sequence,
    // is constrained by this formula:
    // Frame_Period_Max_Bound >= Frame_Period_Min_Bound + Shutter_Max_Bound    
    uint16_t shutterMaxBound = 0x100;  // default value = 0x4e20, 0x100 allows 11748 fps tracking but requires better surface quality
    modifyReg16("Shutter_Max_Bound", REG_Shutter_Max_Bound_Upper, shutterMaxBound);
    modifyReg16("Frame_Period_Min_Bound", REG_Frame_Period_Min_Bound_Upper, 0x0fa0); // 0x0fa0 is the minimal allowed value
    // Set upper frame bound (default 0x5dc0 = 0x4e20 + 0x0fa0)
    // This register must be written last. This write also activates Shutter_Max_Bound and Frame_Period_Min_Bound settings.
    modifyReg16("Frame_Period_Max_Bound", REG_Frame_Period_Max_Bound_Upper, 0x0fa0 + shutterMaxBound);
    // Must seriously wait after setting this register
    delay(2);
}

// Perform complete initialization of ADNS-9800
int performStartup (void)
{
    adns_com();
    adns_com();
    adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
    delay(50); // wait for it to reboot
    // read registers 0x02 to 0x06 (and discard the data)
    adns_read_reg(REG_Motion);
    adns_read_reg(REG_Delta_X_L);
    adns_read_reg(REG_Delta_X_H);
    adns_read_reg(REG_Delta_Y_L);
    adns_read_reg(REG_Delta_Y_H);
    // upload the firmware
    adns_upload_firmware();
    delay(10);
    if (adns_check_firmware() != 0) {
      Serial.println("SROM CRC error");
      return -1;
    }
        
    dispRegisters();
    configureSensor();    
    
    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    uint8_t laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
    modifyReg("LASER_CTRL0", REG_LASER_CTRL0, (laser_ctrl0 & 0xf0) | 0x04); // CW mode, laser enabled    
    delay(1);

    Serial.println("\nOptical Chip Initialized");
    return 0;
}

// Poll motion data using Motion_Burst. Return true if there is a motion update.
int UpdatePointer(void)
{
    uint8_t motion, dx_l, dx_h, dy_h, dy_l;
    adns_com(
      SPI.transfer(REG_Motion_Burst);
      delayMicroseconds(200);
      do {
          motion = SPI.transfer(0);
          if (motion & 0x80 == 0)
            break;
          /* Observation*/ SPI.transfer(0);
          dx_l   = SPI.transfer(0);
          dx_h   = SPI.transfer(0);
          dy_l   = SPI.transfer(0);
          dy_h   = SPI.transfer(0);
          squal  = SPI.transfer(0) << 2;
          /* Pixel_Sum */     SPI.transfer(0);
          /* Maximum_Pixel */ SPI.transfer(0);
          /* Minimum_Pixel */ SPI.transfer(0);
          /* Shutter_Upper */ SPI.transfer(0);
          /* Shutter_Lower */ SPI.transfer(0);
          frameRate = 50000000L / ((SPI.transfer(0) << 8) + SPI.transfer(0));
      } while(0);
    );
    
    rel_x = (int16_t) ((dx_h << 8) | dx_l);
    rel_y = (int16_t) ((dy_h << 8) | dy_l);
    
    abs_x += rel_x;
    abs_y += rel_y;
    
    return motion & 0x80;    
}

void dispRegisters(void){
    int oreg[] = {0x00, 0x3F, 0x2A, 0x02};
    char* oregname[] = {"Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"};
    
    adns_com(  
      for(int rctr=0; rctr < sizeof(oreg) / sizeof(oreg[0]); rctr++) {
          SPI.transfer(oreg[rctr]);
          delay(1);
          Serial.print(oreg[rctr], HEX); Serial.print(" ");
          Serial.print(oregname[rctr]); Serial.print(" ");
          Serial.println(SPI.transfer(0), HEX);  
          delay(1);
      }
    );
}

uint8_t adns_read_reg(uint8_t reg_addr)
{
    adns_com(
      // send adress of the register, with MSBit = 0 to indicate it's a read
      SPI.transfer(reg_addr & 0x7f);
      delayMicroseconds(100); // tSRAD
      // read data
      byte data = SPI.transfer(0);
      
      delayMicroseconds(1); // tSCLK-ADNS_CS for read operation is 120ns
    );
    delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-ADNS_CS
    
    return data;
}

void adns_write_reg(uint8_t reg_addr, uint8_t data)
{
    adns_com(    
      //send adress of the register, with MSBit = 1 to indicate it's a write
      SPI.transfer(reg_addr | 0x80);
      //sent data
      SPI.transfer(data);
      
      delayMicroseconds(20); // tSCLK-ADNS_CS for write operation
    );
    delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-ADNS_CS. Could be shortened, but is looks like a safe lower bound 
}

// Upload firmware to ADNS-9800
void adns_upload_firmware()
{
    // send the firmware to the chip, cf p.18 of the datasheet
    Serial.println("Uploading firmware...");
    // set the configuration_IV register in 3k firmware mode
    adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
    
    // write 0x1d in SROM_enable reg for initializing
    adns_write_reg(REG_SROM_Enable, 0x1d); 
    
    // wait for more than one frame period
    delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
    
    // write 0x18 to SROM_enable to start SROM download
    adns_write_reg(REG_SROM_Enable, 0x18); 
    
    // write the SROM file (=firmware data) 
    adns_com(
      SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
      delayMicroseconds(15);
      
      // send all bytes of the firmware
      for (int i = 0; i < sizeof(SROMA6)/sizeof(SROMA6[0]); i++){ 
          SPI.transfer(SROMA6[i]);
          delayMicroseconds(15);
      }
    );    
}

// Calculate firmware CRC, return 0 if it's correct
int adns_check_firmware(void)
{
    adns_write_reg(REG_SROM_Enable, 0x15);
    delay(10);
    Serial.print("Firmware CRC: ");
    uint16_t crc = (uint16_t) adns_read_reg(REG_Data_Out_Lower) | adns_read_reg(REG_Data_Out_Upper) << 8;
    Serial.println(crc, HEX);
    
    return crc != 0xBEEF;
}

// Verbosely read and modify a 8-bit register. Return value read back from the register after the operation.
uint8_t modifyReg(const char* name, uint8_t addr, uint8_t newvalue)
{
    Serial.print(name); Serial.print(" "); Serial.print(adns_read_reg(addr), HEX); 
    adns_write_reg(addr, newvalue);
    Serial.print(" -> "); 
    uint8_t readback = adns_read_reg(addr);
    Serial.println(readback, HEX);
    
    return readback;
}

// Verbosely read and modify a 16-bit register. Return value read back from the register after the operation.
uint16_t modifyReg16(const char* name, uint8_t addr_upper, uint16_t newvalue)
{
    Serial.print(name); Serial.print(" ");
    uint16_t oldval = (adns_read_reg(addr_upper) << 8) + adns_read_reg(addr_upper - 1);
    adns_write_reg(addr_upper - 1, newvalue & 0xff);
    adns_write_reg(addr_upper, newvalue >> 8);
    Serial.print(oldval, HEX);
    uint16_t readback = (adns_read_reg(addr_upper) << 8) + adns_read_reg(addr_upper - 1);
    Serial.print(" -> "); Serial.println(readback, HEX);
    
    return readback;
}

