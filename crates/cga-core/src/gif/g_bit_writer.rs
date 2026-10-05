pub(crate) struct GBitWriter {
    pub(crate) out: Vec<u8>,
    pub(crate) bits: u32,
    pub(crate) nbits: i32,
}
impl GBitWriter {
    pub(crate) fn write(&mut self, code: u32, width: i32) {
        self.bits |= code << self.nbits;
        self.nbits += width;
        while self.nbits >= 8 {
            self.out.push((self.bits & 0xFF) as u8);
            self.bits >>= 8;
            self.nbits -= 8;
        }
    }
}
