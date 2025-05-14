function decodeUplink(input) {
  const bytes = input.bytes;

  if (bytes.length !== 11) {
    return { errors: ["Invalid payload length for workmode 101"] };
  }

  const bat_mv = (bytes[0] << 8) | bytes[1];

  // --- Decode SIGNED 24-bit int from 3 bytes (big-endian) ---
  let adc = (bytes[2] << 16) | (bytes[3] << 8) | bytes[4];
  // If sign bit is set, extend the sign to 32-bit signed int
  if (adc & 0x800000) {
    adc |= 0xFF000000;  // Sign extension for 32-bit
  }
  adc = adc << 0;  // Force to signed 32-bit

  // Convert to voltage assuming ±VREF (e.g., 11V full scale)
  const pdm = (adc / Math.pow(2, 23)) * 11;

  const temp_raw = (bytes[5] << 8) | bytes[6];
  const temp1 = (temp_raw & 0x8000) ? temp_raw - 0x10000 : temp_raw;
  const temp1_c = temp1 / 10;

  return {
    data: {
      bat_mv,
      ADC_ext_24bit: adc,
      pdm,
      temp1_c
    }
  };
}
