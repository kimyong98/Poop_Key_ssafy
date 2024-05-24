package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.User;

public interface UserDao {
    public void insert(User user);

    public User selectOne(String email);
}
