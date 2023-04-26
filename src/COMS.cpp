

void sendStatus() {
    WebSerial.print(String(millis() - time0));
    WebSerial.print(" stand=");
    WebSerial.print(standing);
    WebSerial.print(" accX=");
    WebSerial.print(accXdata);
    WebSerial.print(" power=");
    WebSerial.print(power);
    WebSerial.print(" ang=");
    WebSerial.print(varAng);
    WebSerial.print(", ");
    WebSerial.print(String(millis() - time0));
    WebSerial.println();
}