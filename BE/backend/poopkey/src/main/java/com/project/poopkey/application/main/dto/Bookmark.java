package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class Bookmark {
    private long bookmarkId;
    private String content;
    private int userId;
}
