import org.gradle.api.tasks.testing.logging.TestExceptionFormat
import java.util.Locale

plugins {
    id("java-library")
	id("com.github.ben-manes.versions") version "0.48.0"
    id("io.github.krakowski.jextract") version "0.3.2"
    id("maven-publish")
    signing
}

repositories {
    mavenCentral()
}

group = "io.calimero"
version = "0.1-SNAPSHOT"

extra["junitJupiterVersion"] = "5.10.0"

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(20))
    }
    withSourcesJar()
    withJavadocJar()
}

tasks.compileJava { options.encoding = "UTF-8" }
tasks.compileTestJava { options.encoding = "UTF-8" }
tasks.javadoc { options.encoding = "UTF-8" }

tasks.compileJava {
    options.compilerArgs = listOf(
        "-Xlint:all,-serial",
        "--limit-modules", "java.base",
        "--enable-preview"
    )
}

tasks.compileTestJava {
    options.compilerArgs = listOf(
        "-Xlint:all",
        "-Xlint:-try",
        "--enable-preview")
}

tasks.withType<Javadoc> {
    val docOpts = (options as StandardJavadocDocletOptions)
    docOpts.addStringOption("Xdoclint:-missing", "-quiet")
    docOpts.addBooleanOption("-enable-preview", true)
    docOpts.addStringOption("-release", java.toolchain.languageVersion.get().toString())
}

tasks.withType<Jar> {
	from("${projectDir}/../LICENSE") {
        into("META-INF")
    }
    if (name == "sourcesJar") {
    	duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    	from("$projectDir") {
		    include("README.md")
        }
    }
    archiveBaseName.set(rootProject.name)
}

testing {
    suites {
        // Configure the built-in test suite
        val test by getting(JvmTestSuite::class) {
            useJUnitJupiter(project.extra["junitJupiterVersion"].toString())
            targets.all {
                testTask.configure {
                    jvmArgs(listOf("--enable-preview"))
                }
            }
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

project.gradle.startParameter.excludedTaskNames.add("jextract")

tasks.jextract {
	val os = System.getProperty("os.name").lowercase(Locale.ENGLISH)

	var headersDir = "unix"
	val path: String
    var tgtPkg = "org.unix"
	var clsName = "Linux"

	if (os.contains("windows")) {
		headersDir = "win"
		path = "C:/mingw64/x86_64-w64-mingw32/include"
		tgtPkg = "org.win"
		clsName = "Windows"
	}
	else if (os.contains("linux")) { 
		path = "/usr/include"
	}
	else if (os.contains("mac")) {
		path = "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include"
	}
	else {
		throw RuntimeException("Unsupported platform \"$os\"")
	}

	header("${project.projectDir}/src/include/$headersDir/headers.h") {
		targetPackage.set(tgtPkg)
		className.set(clsName)
		includes.set(listOf(path))
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
                description.set("Serial comummication using the Foreign Function and Memory API")
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
