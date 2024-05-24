import { useHistory } from "react-router-dom";
import useFetch from "../hooks/useFetch"

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    const history = useHistory(); 
    function addDay() {
        fetch(`http://localhost:3001/days/`, {
            //  주소는 ${props.word.id} 제거 해 주자.
            method : 'POST',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                day : days.length + 1,
            }),
        })
        .then(res=>{
            if(res.ok) {
                alert("단어 추가 완료!");
                // 해당 페이지로 이동 Link to와 유사하다.
                history.push(`/`);
            }
        });
    }

    return <>
        <div>현재 일수 : {days.length}일</div>
        <button onClick={addDay}>Day 추가</button>
    </>
}