pub(crate) struct Rng {
    s: u64,
}
impl Rng {
    pub(crate) fn new(seed: u64) -> Rng {
        Rng { s: seed }
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub(crate) fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / 9_007_199_254_740_992.0
    }

    pub(crate) fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    pub(crate) fn below(&mut self, n: u64) -> u64 {
        self.next_u64() % n
    }
}
