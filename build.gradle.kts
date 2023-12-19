import org.gradle.api.tasks.testing.logging.TestExceptionFormat
import java.util.Locale

plugins {
    id("java-library")
	id("com.github.ben-manes.versions") version "0.50.0"
    id("io.github.krakowski.jextract") version "0.4.1"
    id("maven-publish")
    signing
}

repositories {
    mavenCentral()
}

group = "io.calimero"
version = "0.1-SNAPSHOT"

extra["junitJupiterVersion"] = "5.10.1"

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(22))
    }
    withSourcesJar()
    withJavadocJar()
}

tasks.compileJava { options.encoding = "UTF-8" }
tasks.compileTestJava { options.encoding = "UTF-8" }
tasks.javadoc { options.encoding = "UTF-8" }

tasks.compileJava {
    options.compilerArgs = listOf(
        "-Xlint:all,-serial,-restricted",
        "--limit-modules", "java.base"
    )
}

tasks.compileTestJava {
    options.compilerArgs = listOf(
        "-Xlint:all",
        "-Xlint:-try")
}

tasks.withType<Javadoc> {
    val docOpts = (options as StandardJavadocDocletOptions)
    docOpts.addStringOption("Xdoclint:-missing", "-quiet")
    docOpts.addStringOption("-release", java.toolchain.languageVersion.get().toString())
}

tasks.withType<Jar> {
	from("${projectDir}/LICENSE") {
        into("META-INF")
    }
    if (name == "sourcesJar") {
    	duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    	from("$projectDir") {
		    include("README.md")
        }
    }
//    archiveBaseName.set(rootProject.name)
}

testing {
    suites {
        // Configure the built-in test suite
        val test by getting(JvmTestSuite::class) {
            useJUnitJupiter(project.extra["junitJupiterVersion"].toString())
        }
    }
}

tasks.test {
    testLogging {
        showStandardStreams = true
        exceptionFormat = TestExceptionFormat.FULL
    }

 	beforeTest(closureOf<TestDescriptor> {
        println("$name started")
    })

	afterTest(closureOf<TestDescriptor> {
		println("$name finished")
	})
}

//project.gradle.startParameter.excludedTaskNames.add("jextract")

tasks.jextract {
	val os = System.getProperty("os.name").lowercase(Locale.ENGLISH)

	var headersDir = "unix"
	val paths: List<String>
    var tgtPkg = "org.unix"
	var clsName = "Linux"
	// filters for extracted header file definitions
    var funcFilter = listOf<String>()
	var structFilter = listOf<String>()
	var constantFilter = listOf<String>()

    when {
        os.contains("windows") -> {
            headersDir = "win"
//            paths = listOf("C:/mingw64/x86_64-w64-mingw32/include")
            val sdkDir = "C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0"
            paths = listOf("$sdkDir/um", "$sdkDir/shared", "$sdkDir/ucrt")
            tgtPkg = "org.win"
            clsName = "Windows"

            funcFilter = listOf("RegOpenKeyExA", "RegEnumValueA", "RegCloseKey",
                "GetFileType", "CloseHandle", "CreateFileA", "ReadFile", "WriteFile", "FlushFileBuffers", "GetLastError",
                "SetupComm", "GetCommProperties", "EscapeCommFunction", "GetCommState", "SetCommState", "ClearCommError",
                "GetCommMask", "GetCommModemStatus", "SetCommMask", "WaitCommEvent", "GetCommTimeouts", "SetCommTimeouts",
                "FormatMessageA", "GetOverlappedResult", "CreateEventA", "WaitForSingleObject")
            structFilter = listOf("HKEY__", "_COMMPROP", "_COMMTIMEOUTS", "_COMSTAT", "_DCB", "_OVERLAPPED")
            constantFilter = listOf("FALSE", "TRUE", "NULL",
                "HKEY_LOCAL_MACHINE", "KEY_READ",
                "FILE_TYPE_CHAR", "GENERIC_READ", "GENERIC_WRITE", "FILE_ATTRIBUTE_NORMAL", "OPEN_EXISTING", "FILE_FLAG_OVERLAPPED",
                "WAIT_OBJECT_0", "INFINITE",
                "NO_ERROR", "ERROR_SUCCESS", "ERROR_PATH_NOT_FOUND", "ERROR_NO_MORE_ITEMS", "ERROR_IO_PENDING",
                "INVALID_HANDLE_VALUE", "FILE_TYPE_UNKNOWN", "ERROR_FILE_NOT_FOUND",
                "NOPARITY", "ONESTOPBIT", "TWOSTOPBITS",
                "RTS_CONTROL_HANDSHAKE", "RTS_CONTROL_DISABLE", "DTR_CONTROL_DISABLE", "SETDTR",
                "EV_RLSD", "EV_DSR", "EV_RXCHAR", "EV_RXFLAG", "EV_CTS", "EV_BREAK", "EV_RING", "EV_ERR",
                "CE_BREAK", "CE_RXOVER", "CE_OVERRUN", "CE_RXPARITY", "CE_FRAME",
                "FORMAT_MESSAGE_FROM_SYSTEM", "FORMAT_MESSAGE_IGNORE_INSERTS", "FORMAT_MESSAGE_MAX_WIDTH_MASK")
        }
        os.contains("linux") -> paths = listOf("/usr/include")
        os.contains("mac") ->
            paths = listOf("/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include")
        else -> throw RuntimeException("Unsupported platform \"$os\"")
    }

	header("${project.projectDir}/src/include/$headersDir/headers.h") {
		targetPackage.set(tgtPkg)
		className.set(clsName)
		includes.set(paths)
        functions.set(funcFilter)
        structs.set(structFilter)
        constants.set(constantFilter)
	}
}

dependencies {
    testImplementation("org.junit.jupiter:junit-jupiter:${project.extra["junitJupiterVersion"]}")

    testRuntimeOnly("org.slf4j:slf4j-jdk-platform-logging:2.0.9")
    testRuntimeOnly("org.slf4j:slf4j-simple:2.0.9")
}

publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            artifactId = rootProject.name
            from(components["java"])
            pom {
                name.set("Serial communication FFM")
                description.set("Serial communication using the Foreign Function and Memory API")
                url.set("https://github.com/calimero-project/serial-ffm")
                inceptionYear.set("2022")
                licenses {
                    license {
                        name.set("MIT License")
                        url.set("LICENSE")
                    }
                }
                developers {
                    developer {
                        name.set("Boris Malinowsky")
                        email.set("b.malinowsky@gmail.com")
                    }
                }
                scm {
                    connection.set("scm:git:git://github.com:calimero-project/serial-ffm.git")
                    url.set("https://github.com/calimero-project/serial-ffm.git")
                }
            }
        }
    }
    repositories {
        maven {
            name = "maven"
            val releasesRepoUrl = "https://s01.oss.sonatype.org/service/local/staging/deploy/maven2/"
            val snapshotsRepoUrl = "https://s01.oss.sonatype.org/content/repositories/snapshots/"
            url = uri(if (version.toString().endsWith("SNAPSHOT")) snapshotsRepoUrl else releasesRepoUrl)
            credentials(PasswordCredentials::class)
        }
    }
}

signing {
    if (project.hasProperty("signing.keyId")) {
        sign(publishing.publications["mavenJava"])
    }
}
