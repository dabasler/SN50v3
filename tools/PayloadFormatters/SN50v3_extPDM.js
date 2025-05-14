function decodeUplink(input) {
  const bytes = input.bytes;

  if (bytes.length !== 17) {
    return { errors: ["Invalid payload length for workmode 102"] };
  }

  const bat_mv = (bytes[0] << 8) | bytes[1];

  // 24-bit unsigned ADC value
  const adc = (bytes[2] << 16) | (bytes[3] << 8) | bytes[4];
  const pdm = (adc / Math.pow(2, 24)) * 11;

  // Temperature (SHT) - signed 16-bit
  const temp_raw = (bytes[5] << 8) | bytes[6];
  const temp_sht = (temp_raw & 0x8000) ? temp_raw - 0x10000 : temp_raw;
  const temp_c = temp_sht / 10;

  // Humidity - signed 16-bit
  const hum_raw = (bytes[7] << 8) | bytes[8];
  const hum_sht = (hum_raw & 0x8000) ? hum_raw - 0x10000 : hum_raw;
  const humidity = hum_sht / 10;

  // Unpack 64-bit RGBIR with BigInt-safe shifts
  const rgbir =
    (BigInt(bytes[9])  << BigInt(56)) |
    (BigInt(bytes[10]) << BigInt(48)) |
    (BigInt(bytes[11]) << BigInt(40)) |
    (BigInt(bytes[12]) << BigInt(32)) |
    (BigInt(bytes[13]) << BigInt(24)) |
    (BigInt(bytes[14]) << BigInt(16)) |
    (BigInt(bytes[15]) << BigInt(8))  |
    BigInt(bytes[16]);

  const gain  = Number((rgbir >> BigInt(60)) & BigInt(0x0F));
  const red   = Number((rgbir >> BigInt(48)) & BigInt(0xFFFF));
  const green = Number((rgbir >> BigInt(32)) & BigInt(0xFFFF));
  const blue  = Number((rgbir >> BigInt(16)) & BigInt(0xFFFF));
  const ir    = Number(rgbir & BigInt(0xFFFF));

  return {
    data: {
      bat_mv,
      ADC_ext_24bit: adc,
      pdm,
      temp_sht_c: temp_c,
      humidity_percent: humidity,
      gain,
      red,
      green,
      blue,
      ir
    }
  };
}
