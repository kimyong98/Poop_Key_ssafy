package com.project.poopkey;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.security.servlet.SecurityAutoConfiguration;
import org.springframework.context.annotation.ComponentScan;

@ComponentScan(basePackages = {"com.project.poopkey.netty","com.project.poopkey.clientsocket", "com.project.poopkey.application.main.security"})
//@SpringBootApplication(exclude = SecurityAutoConfiguration.class) // 스프링 부트 임시 해제
@SpringBootApplication
public class PoopkeyApplication {

	public static void main(String[] args) {
		SpringApplication.run(PoopkeyApplication.class, args);
	}

}
