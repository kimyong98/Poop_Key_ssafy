import { useHistory } from "react-router-dom";
import useFetch from "../hooks/useFetch"
import { useRef, useState } from "react";

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    const history = useHistory();
    const [isLoading, setIsLoading] = useState(false);

    function onSubmit(e) {
        e.preventDefault();

        if(!isLoading) {
            setIsLoading(true);


            // form 내부의 버튼을 눌렀을 때 새로고침 막는거

            // 이제 각 ref에 잘 담기는지 확인하자
            console.log(engRef.current.value)
            console.log(korRef.current.value)
            console.log(dayRef.current.value)
            // 잘 나오는걸 확인 할 수 있다.


            // Word.js에서 PUT에 해당하는 부분을 긁어와 수정하겠다.
            // POST 코드
            fetch(`http://localhost:3001/words/`, {
                //  주소는 ${props.word.id} 제거 해 주자.
                method : 'POST',
                headers : {
                    'Content-Type' : 'application/json',
                    // Content-Type은 보내는 리소스의 타입을 의미함.
                    // 타입을 json으로 보내주겠음.
                    // 타입은 이미지 json 등등 다양함
                },
                body : JSON.stringify({
                    day : dayRef.current.value,
                    eng : engRef.current.value,
                    kor : korRef.current.value,
                    isDone : false,
                }),
            })
            .then(res=>{
                if(res.ok) {
                    alert("단어 추가 완료!");
                    // 해당 페이지로 이동 Link to와 유사하다.
                    history.push(`/day/${dayRef.current.value}`);
                    setIsLoading(false);
                }
            });
        }
    }



    

    const engRef = useRef(null);
    const korRef = useRef(null);
    const dayRef = useRef(null);
    // useRef란?
    // DOM에 접근하게 해준다 (querySelector 같은거)
    // 스크롤 위치를 확인하거나 포커를 주거나 할 때 사용함



    return <form onSubmit={onSubmit}>
        <div className="input_area">
            <label>Eng</label>
            {/* 각 입력부분마다 ref를 연결한다. */}
            <input type="text" placeholder="computer" ref={engRef}></input>
        </div>

        <div className="input_area">
            <label>Kor</label>
            <input type="text" placeholder="컴퓨터" ref={korRef}></input>
        </div>

        <div className="input_area">
            <label>Day</label>
            {/* select - option 기억하자 */}
            <select ref={dayRef}>
                {days.map(day=>(
                    <option key={day.id} value={day.day}>{day.day}</option>
                ))}
            </select>
            {/*  map(day=>())) =>다음에 ()이야!!! */}
        </div>
        <button>{isLoading ? "Now Saving..." : "저장"}</button>
    </form>
}