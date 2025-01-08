
#include "esp_pn532.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include <PN532_SPI.h>
#include "PN532.h"
#include <cstring>
#define TAG "main"


ESP_PN532::ESP_PN532(gpio_num_t cs_pin, spi_host_device_t spi_host) : pn532_spi(cs_pin, spi_host), nfc(pn532_spi) {
    // Any initialization code that doesn't involve member initialization 
    // can be placed here if needed
}


void ESP_PN532::setup(void) {
  // has to be fast to dump the entire memory contents!
  ESP_LOGI(TAG, "Looking for PN532...");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    ESP_LOGI(TAG, "Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  ESP_LOGI(TAG, "Found chip PN5"); ESP_LOGI(TAG, "Version data: 0x%02X", static_cast<unsigned int>((versiondata >> 24) & 0xFF));
  ESP_LOGI(TAG, "Firmware ver. "); ESP_LOGI(TAG, "Version data: 0x%02X", static_cast<unsigned int>((versiondata>>16) & 0xFF));
  printf("."); ESP_LOGI(TAG, "Version data: 0x%02X", static_cast<unsigned int>((versiondata>>8) & 0xFF));

  // configure board to read RFID tags
  nfc.SAMConfig();
}


void ESP_PN532::read_card(void) {
  uint8_t success;                          // Flag to check if there was an error with the PN532
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  uint8_t currentblock;                     // Counter to keep track of which block we're on
  bool authenticated = false;               // Flag to indicate if the sector is authenticated
  uint8_t data[16];                         // Array to store block data during reads

  // ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

  // Keyb on NDEF and Mifare Classic should be the same
  uint8_t keyuniversal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card
    ESP_LOGI(TAG, "Found an ISO14443A card");
    printf("  UID Length: ");printf("%d", uidLength);printf(" bytes");
    printf("\n  UID Value: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      log_hex(uid[i]);
      printf(" ");
    }
    ESP_LOGI(TAG, "");

    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ...
      ESP_LOGI(TAG, "Seems to be a Mifare Classic card (4 byte UID)");

      // Now we try to go through all 16 sectors (each having 4 blocks)
      // authenticating each sector, and then dumping the blocks
      for (currentblock = 0; currentblock < 64; currentblock++)
      {
        // Check if this is a new block so that we can reauthenticate
        if (nfc.mifareclassic_IsFirstBlock(currentblock)) authenticated = false;

        // If the sector hasn't been authenticated, do so first
        if (!authenticated)
        {
          // Starting of a new sector ... try to to authenticate
          printf("------------------------Sector ");printf("%d", (currentblock/4));printf("-------------------------\n");
          if (currentblock == 0)
          {
              // This will be 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF for Mifare Classic (non-NDEF!)
              // or 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 for NDEF formatted cards using key a,
              // but keyb should be the same for both (0xFF 0xFF 0xFF 0xFF 0xFF 0xFF)
              success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, currentblock, 1, keyuniversal);
          }
          else
          {
              // This will be 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF for Mifare Classic (non-NDEF!)
              // or 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 for NDEF formatted cards using key a,
              // but keyb should be the same for both (0xFF 0xFF 0xFF 0xFF 0xFF 0xFF)
              success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, currentblock, 1, keyuniversal);
          }
          if (success)
          {
            authenticated = true;
          }
          else
          {
            ESP_LOGI(TAG, "Authentication error");
          }
        }
        // If we're still not authenticated just skip the block
        if (!authenticated)
        {
          printf("Block ");printf("%d", currentblock);ESP_LOGI(TAG, " unable to authenticate");
        }
        else
        {
          // Authenticated ... we should be able to read the block now
          // Dump the data into the 'data' array
          success = nfc.mifareclassic_ReadDataBlock(currentblock, data);
          if (success)
          {
            // Read successful
            printf("Block ");printf("%d:", currentblock);
            if (currentblock < 10)
            {
              printf("  ");
            }
            else
            {
              printf(" ");
            }
            // Dump the raw data
            nfc.PrintHexChar(data, 16);
          }
          else
          {
            // Oops ... something happened
            printf("Block ");printf("%d", currentblock);
            ESP_LOGI(TAG, " unable to read this block");
          }
        }
      }
    }
    else
    {
      ESP_LOGI(TAG, "Ooops ... this doesn't seem to be a Mifare Classic card!");
    }
  }
  // Wait a bit before trying again
  ESP_LOGI(TAG, "\n\nSend a character to run the mem dumper again!");
  // Serial.flush();
  // while (!Serial.available());
  // while (Serial.available()) {
  // Serial.read();
  // }
  // Serial.flush();
}

void ESP_PN532::format_card(void) {

  const char * url = "bunny";
  uint8_t ndefprefix = NDEF_URIPREFIX_NONE;


  uint8_t success;                          // Flag to check if there was an error with the PN532
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  bool authenticated = false;               // Flag to indicate if the sector is authenticated

  // Use the default key
  uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  // uint8_t keya[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  printf("\n");
  printf("PLEASE NOTE: Formatting your card for NDEF records will change the\n");
  printf("authentication keys.  To reformat your NDEF tag as a clean Mifare\n");
  printf("Classic tag, use the mifareclassic_ndeftoclassic example!\n");
  printf("\n");
  printf("Place your Mifare Classic card on the reader to format with NDEF\n");
  printf("and press any key to continue ...\n");
  // Wait for user input before proceeding
  // while (!Serial.available());
  // a key was pressed1
  // while (Serial.available()) Serial.read();

  // Wait for an ISO14443A type card (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success)
  {
    // Display some basic information about the card
    ESP_LOGI(TAG, "Found an ISO14443A card");
    printf("  UID Length: ");printf("%d", uidLength);printf(" bytes\n");
    printf("  UID Value: \n");
    nfc.PrintHex(uid, uidLength);
    for (uint8_t i = 0; i < uidLength; i++) {
      log_hex(uid[i]);
      printf(" ");
    }
    printf("\n");

    // Make sure this is a Mifare Classic card
    if (uidLength != 4)
    {
      ESP_LOGW(TAG, "Ooops ... this doesn't seem to be a Mifare Classic card!");
      return;
    }

    // We probably have a Mifare Classic card ...
    ESP_LOGI(TAG, "Seems to be a Mifare Classic card (4 byte UID)");

    // Try to format the card for NDEF data
    success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, 0, 0, keya);
    if (!success)
    {
      ESP_LOGI(TAG, "Unable to authenticate block 0 to enable card formatting!");
      return;
    }
    success = nfc.mifareclassic_FormatNDEF();
    if (!success)
    {
      ESP_LOGI(TAG, "Unable to format the card for NDEF");
      return;
    }

    ESP_LOGI(TAG, "Card has been formatted for NDEF data using MAD1");

    // Try to authenticate block 4 (first block of sector 1) using our key
    success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, 4, 0, keya);

    // Make sure the authentification process didn't fail
    if (!success)
    {
      ESP_LOGI(TAG, "Authentication failed.");
      return;
    }

    // Try to write a URL
    ESP_LOGI(TAG, "Writing URI to sector 1 as an NDEF Message");

    // Authenticated seems to have worked
    // Try to write an NDEF record to sector 1
    // Use 0x01 for the URI Identifier Code to prepend "http://www."
    // to the url (and save some space).  For information on URI ID Codes
    // see http://www.ladyada.net/wiki/private/articlestaging/nfc/ndef
    if (strlen(url) > 38)
    {
      // The length is also checked in the WriteNDEFURI function, but lets
      // warn users here just in case they change the value and it's bigger
      // than it should be
      ESP_LOGI(TAG, "URI is too long ... must be less than 38 characters long");
      return;
    }

    // URI is within size limits ... write it to the card and report success/failure
    success = nfc.mifareclassic_WriteNDEFURI(1, ndefprefix, url);
    if (success)
    {
      ESP_LOGI(TAG, "NDEF URI Record written to sector 1");
    }
    else
    {
      ESP_LOGI(TAG, "NDEF Record creation failed! :(");
    }
  }

  // Wait a bit before trying again
  ESP_LOGI(TAG, "\nDone!");
  // delay(1000);
  // Serial.flush();
  // while(Serial.available()) Serial.read();
}

int ESP_PN532::read_uid(uint8_t* uid_buffer) {
  uint8_t success;                          // Flag to check if there was an error with the PN532
  // uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

  // Keyb on NDEF and Mifare Classic should be the same
  uint8_t keyuniversal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  memset(uid_buffer, 0, 7);
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid_buffer, &uidLength, 10); //add timeout
  if (success) {
    return 1;
  } else {
    return 0;
  }
}

void ESP_PN532::log_hex(uint8_t value) {
  printf("0x%02X", value);
}

