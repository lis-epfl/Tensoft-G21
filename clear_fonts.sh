#!/usr/bin/env bash

pushd /usr/share/fonts/truetype/
mkdir tmp
mv SourceSansPro/SourceSansPro-Light.ttf SourceSansPro/.uuid tmp
rm -rf SourceSansPro
mv tmp SourceSansPro
popd
