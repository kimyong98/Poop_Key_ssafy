package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.BookmarkDao;
import com.project.poopkey.application.main.dto.Bookmark;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class BookmarkServiceImpl implements BookmarkService{

    @Autowired
    private BookmarkDao bookmarkDao;

    @Override
    public List<Bookmark> findList(int userId) {
        return bookmarkDao.selectList(userId);
    }

    @Override
    public void addBookmark(Bookmark bookmark) {
        bookmarkDao.insertOne(bookmark);
    }
}
