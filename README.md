Serial port communication Java library using the Foreign Function and Memory API
=====

Serial port access for Linux, macOS, and Windows using Java's [Foreign Function and Memory (FFM) API](https://openjdk.org/jeps/442).

[Project Panama](https://openjdk.org/projects/panama/) aims at improving interoperability between Java and native libraries. It provides the new Foreign Function & Memory API, and enables access to native code in pure Java.
The FFM API is available as Java preview feature. 

This library is implemented using Java 21 and requires a JDK 21 for the preview features to work. The implementation is a port of the [serial-native](https://github.com/calimero-project/serial-native) C libraries written to use JNI.

The header files are generated using [jextract](https://github.com/openjdk/jextract), leveraging the [gradle-jextract](https://plugins.gradle.org/plugin/io.github.krakowski.jextract) plugin in the gradle build file.


### Build with Gradle 

    ./gradlew build -x test


### Examples

Using a serial port named _portId_ in a try-with-resources statement:

```
try (var port = SerialPort.open(portId)
		.baudrate(19_200)
		.databits(8)
		.parity(Parity.Even)
		.stopbits(StopBits.One)
		.flowControl(FlowControl.None)) {
	var in = port.inputStream();
	var out = port.outputStream();
	// use streams ...
}
```
