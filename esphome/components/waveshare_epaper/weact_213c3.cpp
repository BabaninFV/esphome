#include "waveshare_epaper.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace waveshare_epaper {

static const char *const TAG = "weact_2.13c3";

static const uint16_t HEIGHT = 250;
static const uint16_t WIDTH = 122;

static const uint8_t PARTIAL_LUT[] = {
    0x32,  // cmd
    0x0,  0x40, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0, 0x40, 0x40, 0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x80, 0x0, 0x0,
    0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0xF,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x4,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0x0,  0x0,  0x1, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x1,  0x0, 0x0, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
};

static const uint8_t FULL_LUT[] = {
    0x32,  // CMD
    0x80, 0x4A, 0x40, 0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x40, 0x4A, 0x80, 0x0, 0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0, 0x80, 0x4A, 0x40, 0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x40, 0x4A, 0x80, 0x0,
    0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,
    0xF,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0xF,  0x0,  0x0,  0xF, 0x0, 0x0,  0x2,  0xF,  0x0, 0x0,  0x0,  0x0,  0x0,
    0x0,  0x1,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x0,  0x0, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
};

static const uint8_t SW_RESET = 0x12;
static const uint8_t ACTIVATE = 0x20;
static const uint8_t WRITE_BUFFER = 0x24;
static const uint8_t WRITE_BASE = 0x26;

static const uint8_t DRV_OUT_CTL[] = {0x01, 0x27, 0x01, 0x01};  // driver output control {0x01, 0x27, 0x01, 0x00(Mirror)}
static const uint8_t GATEV[] = {0x03, 0x17};
static const uint8_t SRCV[] = {0x04, 0x41, 0x0C, 0x32};
static const uint8_t SLEEP[] = {0x10, 0x01};
static const uint8_t DATA_ENTRY[] = {0x11, 0x01};            // data entry mode // {0x11, 0x03}
static const uint8_t TEMP_SENS[] = {0x18, 0x80};             // Temp sensor
static const uint8_t DISPLAY_UPDATE[] = {0x21, 0x00, 0x80};  // Display update control
static const uint8_t POWER_ON[] = {0x22, 0xF8};
static const uint8_t POWER_OFF[] = {0x22, 0x83};
static const uint8_t UPSEQ[] = {0x22, 0xC0};
static const uint8_t ON_FULL[] = {0x22, 0xF7}; // {0x22, 0xC7} // 0xF7 (blinks)
static const uint8_t ON_FULL_FAST[] = {0x22, 0xC7};
static const uint8_t ON_PARTIAL[] = {0x22, 0x0F};
static const uint8_t VCOM[] = {0x2C, 0x36};
static const uint8_t BORDER_PART[] = {0x3C, 0x80};  // border waveform
static const uint8_t BORDER_FULL[] = {0x3C, 0x05};  // border waveform
static const uint8_t CMD1[] = {0x3F, 0x22};
static const uint8_t RAM_X_START[] = {0x44, 0x00, 0x0F};           // set ram_x_address_start_end (121 / 8)
static const uint8_t RAM_Y_START[] = {0x45, 0x27, 0x01, 0x00, 0x00};  // set ram_y_address_start_end (0) {0x45, 0x00, 0x00, 250 - 1, 0}
static const uint8_t RAM_X_POS[] = {0x4E, 0x00};                      // set ram_x_address_counter
static const uint8_t RAM_Y_POS = 0x4F;
// static const uint8_t RAM_Y_POS[] = {0x4F, 0x00, 0x00};        // set ram_y_address_counter
#define SEND(x) this->cmd_data(x, sizeof(x))

/**************************************************************/
/*************************** Basics ***************************/
/**************************************************************/
int WeActEPaper2P13InC3::get_width_controller() { return 128; }
int WeActEPaper2P13InC3::get_width_internal() { return 128; }
int WeActEPaper2P13InC3::get_height_internal() { return 250; }
uint32_t WeActEPaper2P13InC3::idle_timeout_() { return 2500; }

void WeActEPaper2P13InC3::dump_config() {
  LOG_DISPLAY("", "WeAct E-Paper (3 Color)", this)
  ESP_LOGCONFIG(TAG, "  Model: 2.13in Red+Black");
  LOG_PIN("  CS Pin: ", this->cs_)
  LOG_PIN("  Reset Pin: ", this->reset_pin_)
  LOG_PIN("  DC Pin: ", this->dc_pin_)
  LOG_PIN("  Busy Pin: ", this->busy_pin_)
  LOG_UPDATE_INTERVAL(this)
}

/************************************************************************/
/*************************** Device lifecycle ***************************/
/************************************************************************/
void WeActEPaper2P13InC3::setup() {
  this->init_internal_(this->get_buffer_length_());
  this->setup_pins_();
  this->spi_setup();
  this->reset_();

  delay(20);
  this->send_reset_();
  // as a one-off delay this is not worth working around.
  delay(100);  // NOLINT
  this->wait_until_idle_();
  this->command(SW_RESET);
  this->wait_until_idle_();

  SEND(DRV_OUT_CTL);
  SEND(DATA_ENTRY);
  SEND(BORDER_FULL);
  SEND(DISPLAY_UPDATE);
  SEND(TEMP_SENS);

  this->set_window_(0, this->get_height_internal());

  SEND(POWER_ON);
  this->command(ACTIVATE);

  this->wait_until_idle_();
  this->write_lut_(FULL_LUT);
}

void WeActEPaper2P13InC3::send_reset_() {
  ESP_LOGI(TAG, "Resetting e-paper display.");

  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(false);
    delay(2);
    this->reset_pin_->digital_write(true);
  }
}

// must implement, but we override setup to have more control
void WeActEPaper2P13InC3::initialize() {}

/*************************************************************************/
/****************************** Pixel stuff ******************************/
/*************************************************************************/

void WeActEPaper2P13InC3::write_lut_(const uint8_t *lut) {
  this->wait_until_idle_();
  this->cmd_data(lut, sizeof(PARTIAL_LUT));
  SEND(CMD1);
  SEND(GATEV);
  SEND(SRCV);
  SEND(VCOM);
}

// t and b are y positions, i.e. line numbers.
void WeActEPaper2P13InC3::set_window_(int t, int b) {
  SEND(RAM_X_START);
  SEND(RAM_Y_START);
  SEND(RAM_X_POS);

  uint8_t buffer[3];
  buffer[0] = RAM_Y_POS;
  buffer[1] = (uint8_t) (295 & 0xff);       // 295 = 250 height + 45 offset
  buffer[2] = (uint8_t) (295 >> 8 & 0x01);  // 295 = 250 height + 45 offset
  SEND(buffer);
}

// write the buffer starting on line top, up to line bottom.
void WeActEPaper2P13InC3::write_buffer_(int top, int bottom) {
  auto width_bytes = this->get_width_internal() / 8u;
  auto offset = top * width_bytes;
  auto length = (bottom - top) * width_bytes;

  this->wait_until_idle_();
  this->set_window_(top, bottom);

  this->command(WRITE_BUFFER);
  this->start_data_();
  this->write_array(this->buffer_ + offset, length);
  this->end_data_();

  offset += this->get_buffer_length_() / 2u;
  this->command(WRITE_BASE);
  this->start_data_();
  this->write_array(this->buffer_ + offset, length);
  this->end_data_();
}

void WeActEPaper2P13InC3::partial_update_() {
  ESP_LOGI(TAG, "Performing partial e-paper update.");

  this->send_reset_();
  this->set_timeout(100, [this] {
    SEND(BORDER_PART);
    SEND(UPSEQ);
    this->command(ACTIVATE);
    this->set_timeout(100, [this] {
      this->write_buffer_(0, this->get_height_internal());
      SEND(ON_PARTIAL);
      this->command(ACTIVATE);  // Activate Display Update Sequence
      this->is_busy_ = false;
    });
  });
}

void WeActEPaper2P13InC3::full_update_() {
  ESP_LOGI(TAG, "Performing full e-paper update.");
  this->write_lut_(FULL_LUT);
  this->write_buffer_(0, this->get_height_internal());
  SEND(ON_FULL);
  this->command(ACTIVATE);  // don't wait here
  this->is_busy_ = false;
}

void WeActEPaper2P13InC3::display() {
  if (this->is_busy_ || (this->busy_pin_ != nullptr && this->busy_pin_->digital_read()))
    return;

  this->is_busy_ = true;
  // this->full_update_();
  const bool partial = this->at_update_ != 0;
  this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
  if (partial) {
    this->partial_update_();
  } else {
    this->full_update_();
  }
}

void WeActEPaper2P13InC3::deep_sleep() {
  SEND(POWER_OFF);

  // COMMAND POWER DOWN
  this->command(0x10);
  this->data(0x01);
  // cannot wait until idle here, the device no longer responds
}

}  // namespace waveshare_epaper
}  // namespace esphome
