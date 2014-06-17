this SBC decoder derived from Bluez, you can also use it in Bluedroid.
1.if you want to SBC decode, please call the intereface function:
SBC_API extern void SBC_Decoder(SBC_DEC_PARAMS *strDecParams);
2.sbc_debug.c in TEST can help you to verify your decoder, please run it just like usual c programe.