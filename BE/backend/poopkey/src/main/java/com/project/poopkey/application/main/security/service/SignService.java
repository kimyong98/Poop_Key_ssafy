package com.project.poopkey.application.main.security.service;

import com.project.poopkey.application.main.security.JwtTokenProvider;
import com.project.poopkey.application.main.security.dao.MemberDao;
import com.project.poopkey.application.main.security.dto.Member;
import com.project.poopkey.application.main.security.dto.SignRequest;
import com.project.poopkey.application.main.security.dto.SignResponse;
import lombok.RequiredArgsConstructor;
import org.springframework.mail.SimpleMailMessage;
import org.springframework.mail.javamail.JavaMailSender;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.core.Authentication;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.ArrayList;
import java.util.Random;

@Service
@Transactional
@RequiredArgsConstructor
public class SignService {
    private final MemberDao memberDao;
    private final PasswordEncoder passwordEncoder;
    private final JwtTokenProvider jwtTokenProvider;

    // 이메일 인증관련
    private final JavaMailSender javaMailSender;

    public SignResponse login(SignRequest request)throws Exception{
        Member member = memberDao.findByEmail(request.getEmail()).orElseThrow(()->
                new BadCredentialsException("잘못된 계정정보"));
        if(!passwordEncoder.matches(request.getPassword(), member.getPassword())){
            throw new BadCredentialsException("잘못된 계정정보");
        }
        // 디버깅
        Authentication a = jwtTokenProvider.getAuthentication(jwtTokenProvider.createToken(member.getEmail()));
        System.out.println("토큰에서 추출한 권한정보는");
        System.out.println(a.getAuthorities().toString()); // USER
        // 디버깅
        return SignResponse.builder()
                .email(member.getEmail())
                .role("USER")
                .token(jwtTokenProvider.createToken(member.getEmail()))
                .build();
    }

    public boolean register(SignRequest request)throws Exception{
        if(memberDao.findByEmail(request.getEmail()).isPresent()){
            System.out.println("중복임");
            throw new Exception("이미 중복된 이메일이 존재합니다");
        }
        try {
            Member member = Member.builder()
                    .email(request.getEmail())
                    .password(passwordEncoder.encode(request.getPassword()))
                    .build();
            memberDao.insertUser(member);
        }catch (Exception e){
            System.out.println(e.getMessage());
            throw new Exception("잘못된 요청");
        }
        return true;
    }
    public SignResponse getMember(String email) throws Exception{
        Member member = memberDao.findByEmail(email)
                .orElseThrow(()->new Exception("계정을 찾을 수 없음"));
        return new SignResponse(member);
    }


    public int sendMail(String email){
        SimpleMailMessage simpleMailMessage = new SimpleMailMessage();
        simpleMailMessage.setFrom("보낸사람@daum.net"); // 필수
        simpleMailMessage.setTo(email);
        simpleMailMessage.setSubject("이메일 인증");
        simpleMailMessage.setText("인증번호는 아래와 같습니다.");
        Random random = new Random();
        int checkNum = random.nextInt(888888)+111111;
        simpleMailMessage.setText("인증번호는 아래와 같습니다.");
        return -1;
    }
}
