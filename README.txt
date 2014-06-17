Introduction
============

this SBC decoder derived from Bluez, you can also use it in Bluedroid.

Usage
========

1.if you want to SBC decode, please call the intereface function:
  SBC_API extern void SBC_Decoder(SBC_DEC_PARAMS *strDecParams);
  strDecParams will contain the SBC data stream and it's length, then the decoded PCM will be put in s16Pcmsample.
2.sbc_debug.c in TEST can help you to verify your decoder, please run it just like usual c programe.
  ./main.o -f a ../frame2.txt