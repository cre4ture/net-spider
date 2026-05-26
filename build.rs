use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is always set"));
    File::create(out.join("memory.x"))
        .expect("memory.x output file should be writable")
        .write_all(include_bytes!("memory.x"))
        .expect("memory.x should be copied into OUT_DIR");

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
}
