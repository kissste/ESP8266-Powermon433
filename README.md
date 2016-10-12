ESP8266-Powermon433
===========

 ESP8266-Powermon433
 Forked from scruss/Powermon433

 Monitoring 433MHz power monitor products (both the same internally)
 -- Black and Decker EM100B 
 -- BlueLine PowerCost Monitor

 Please read the original Arduino version scruss/Powermon433 first

 Changes made by kissste:
 -- Ported to ESP8266
 -- Added UDP messaging
 -- Added Temperature and Humidity
 -- Tested with CC1101
 -- Python Server side to receive UDP messages and update RRDTOOL database
 -- Python to render RRDTOOL charts
 
 P.S. Use Arduino IDE with target board being set to ESP8266
 