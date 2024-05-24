package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dto.Bookmark;

import java.awt.print.Book;
import java.util.List;

public interface BookmarkService {
    List<Bookmark> findList(int userId);

    void addBookmark(Bookmark bookmark);
}
