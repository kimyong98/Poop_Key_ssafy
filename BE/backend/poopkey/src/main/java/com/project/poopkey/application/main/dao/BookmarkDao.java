package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.Bookmark;

import java.util.List;

public interface BookmarkDao {
    public List<Bookmark> selectList(int userId);
    public void insertOne(Bookmark bookmark);
}
