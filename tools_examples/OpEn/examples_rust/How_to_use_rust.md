# How to use Rust

## Install

- Install Rust from [here](https://www.rust-lang.org/tools/install)

## Usage

- Create a new project by ```cargo new {project_name} --bin```. Then, it will automatically create a new project folder.

- Move to the project folder

- Modify ``Cargo.toml`` to add any required dependancies after ``[dependencies]``, for example,
  - [OpEn](https://alphaville.github.io/optimization-engine/docs/installation#open-in-rust) : ``optimization_engine = "0.6.2"``
  - [Random Number](https://docs.rs/crate/rand/0.3.14): ``rand = "0.3.14"``

- Build by ``cargo build``

- Run by ``cargo run``

- (Optional) Also there are functions for 
  - Test by `cargo test`
  - Documentation by `cargo doc`
  
