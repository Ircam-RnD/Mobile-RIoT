#!/bin/bash

echo "copy js file"
cp ../bundle.js ./www/js/bundle.js

echo "copy css file"
cp ../css/styles.css ./www/css/styles.css

echo "copy assets"
cp -R ../assets ./www/assets

cordova build ios
