import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

description = "A breadth-first search path finder"

plugins {
    `maven-publish`
    kotlin("jvm") version "1.6.10"
    id("me.champeau.gradle.jmh") version "0.5.2"
    id("org.jmailen.kotlinter") version "3.3.0"
}

repositories {
    mavenCentral()
}

dependencies {
    implementation(kotlin("stdlib"))
    jmh("com.fasterxml.jackson.module:jackson-module-kotlin:2.13.1")
    jmh("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.6.0")
    testImplementation("com.fasterxml.jackson.module:jackson-module-kotlin:2.13.1")
    testImplementation("org.junit.jupiter:junit-jupiter:5.8.2")
}

kotlin {
    explicitApi()
}

tasks.withType<KotlinCompile> {
    kotlinOptions {
        jvmTarget = "17"
        freeCompilerArgs = listOf("-XXLanguage:+InlineClasses")
    }
}

tasks.withType<Test> {
    failFast = true
    useJUnitPlatform()
}

jmh {
    profilers = listOf("stack")
}

java {
    withJavadocJar()
    withSourcesJar()
}

publishing {
    repositories {
        maven {
            name = "GitHubPackages"
            url = uri("https://maven.pkg.github.com/blurite/pathfinder")
            credentials {
                username = System.getenv("GITHUB_ACTOR")
                password = System.getenv("GITHUB_TOKEN")
            }
        }
    }
    publications {
        register<MavenPublication>("gpr") {
            from(components["java"])
        }
    }
}
