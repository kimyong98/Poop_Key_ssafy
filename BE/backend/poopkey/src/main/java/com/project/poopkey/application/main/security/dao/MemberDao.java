package com.project.poopkey.application.main.security.dao;

import com.project.poopkey.application.main.security.dto.Member;

import java.util.Optional;

public interface MemberDao {
    public Optional<Member> findByEmail(String email);
    public void insertUser(Member member);
}
