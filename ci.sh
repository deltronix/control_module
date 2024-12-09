#!/bin/sh

set -e

project="test-app"

cleanup() {
  echo "Cleaning up"
  mv Cargo.toml.tmp Cargo.toml
  mv .cargo/config.toml.tmp .cargo/config.toml
}

if [ "$1" = "cleanup" ]; then
  cleanup
  exit 1
fi

echo "Installing necessary tools"
cargo install flip-link sd

echo "Cleaning up old project"
rm -rf "$project"

echo "Creating new project"
# cargo generate -p . --name "$project"
mkdir -p "$project"
cp -r Cargo.toml LICENSE-* src/ rust-toolchain.toml .cargo/ "$project"

echo "Storing current config so that the child project will compile."
mv Cargo.toml Cargo.toml.tmp
mv .cargo/config.toml .cargo/config.toml.tmp

cd "$project"

echo "Performing steps"

sd -s -- '--chip $CHIP' '--chip nRF52840_xxAA' .cargo/config.toml
sd -s '# target = "thumbv7em-none-eabihf"' 'target = "thumbv7em-none-eabihf"' .cargo/config.toml
sd -s '$RTIC_BACKEND' 'thumbv7-backend' Cargo.toml

cargo bbr minimal

cd ..
cleanup

