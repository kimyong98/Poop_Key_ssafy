import { useState, useEffect } from "react";

export default function Word(props) {

    // word를 state로 하겠음
    // 왜냐하면 DELETE를 했을 때, 새로 랜더링해야하는데 state로 하면 편하니까.
    const [word, setWord] = useState(props.word);

    const [isShow, setIsShow] = useState(false);
    const [isDone, setIsDone] = useState(props.word.isDone);
    // state는 해당 컴포넌트에 있는걸 바꿔야지 props는 건드는거 아님 ㅎㅎ

    function toggleShow() {
        setIsShow(!isShow);
    }

    function toggleDone() { // 여길 바꾸겠음
        // setIsDone(!isDone);
        fetch(`http://localhost:3001/words/${props.word.id}`, {
            method : 'PUT',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                ...props.word,
                isDone : !isDone,
            }),
        })
        .then(res=>{
            if(res.ok) {
                setIsDone(!isDone);
            }
        });
    }

    function del() {
        if(window.confirm('진짜 삭제할꺼야?')) {
            fetch(`http://localhost:3001/words/${props.word.id}`, {
                method : 'DELETE',
            }).then(res=>{
                // .then 이후는 새로 랜더링하기 위한거임
                if(res.ok) {
                    setWord({id:0});
                }
            })

        }
    }

    if (word.id === 0) { // id가 0이면 null을 하는데 null이면 아래 return을 안 해서 아무것도 안뜸.
        return null;
    }

    return <>
    <tr className={isDone ? "off" : ""} key={props.word.id}>
        <td>
            <input type="checkbox" checked={isDone}
            onChange={toggleDone}></input>
        </td>
        <td>
            {props.word.eng}
        </td>
        <td>
            {isShow && props.word.kor}
            {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
        </td>
        <td>
            <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
            {/* 이벤트 onClick 꼭 기억하자 */}
            {/* isShow가 true면 숨기기로 나오겠다. */}
            <button className="btn_del" onClick={del}>삭제</button>
        </td>
    </tr>
    </>;
}




