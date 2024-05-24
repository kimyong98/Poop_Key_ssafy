package com.project.poopkey.application.main.security.service;

import com.project.poopkey.application.main.security.CustomUserDetails;
import com.project.poopkey.application.main.security.dao.MemberDao;
import com.project.poopkey.application.main.security.dto.Member;
import lombok.RequiredArgsConstructor;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class UserDetailsServiceImpl implements UserDetailsService {
    private final MemberDao memberDao;
    @Override
    public UserDetails loadUserByUsername(String username) throws UsernameNotFoundException {
        Member member = memberDao.findByEmail(username).orElseThrow(
                ()->new UsernameNotFoundException("유효하지 않은 인증!")
        );

        return new CustomUserDetails(member);
    }
}
