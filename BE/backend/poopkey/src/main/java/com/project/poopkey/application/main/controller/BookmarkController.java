package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Bookmark;
import com.project.poopkey.application.main.dto.Report;
import com.project.poopkey.application.main.service.BookmarkService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class BookmarkController {
    @Autowired
    private BookmarkService bookmarkService;
    // http://{SERVER_URL}/api/user/bookmark/select
    @GetMapping("/user/bookmark/select")
    @Operation(summary = "북마크 리스트 확인", description = "해당 사용자의 모든 북마크 리스트를 반환")
    public ResponseEntity<?> bookmarkFind(@RequestParam("userId")int userId){
        List<Bookmark> list = bookmarkService.findList(userId);
        if(list==null || list.size()==0)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<List<Bookmark>>(list, HttpStatus.OK);
    }
    @PostMapping("/user/bookmark/insert")
    @Operation(summary = "북마크 추가", description = "사용자가 입력한 북마크를 DB에 추가함")
    public ResponseEntity<Void> bookmarkAdd(@RequestBody Bookmark bookmark){
        bookmarkService.addBookmark(bookmark);
        return new ResponseEntity<Void>(HttpStatus.CREATED);
    }
}
