import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
import useFetch from "../hooks/useFetch";
// import dummy from '../DB/data.json'

export default function DayList() {
    const days = useFetch("http://localhost:3001/days");

    if(days.length === 0) {
        return <span>Now Loading...</span>
    }
    
    // const [days, setDays] = useState([])



    // useEffect(()=>{
    //     fetch('http://localhost:3001/days')
    //     // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
    //     .then(res=>{
    //         // fetch가 되면 == .then
    //         return res.json()
    //         // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
    //         //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
    //     })
    //     .then(data=>{
    //         // 위에걸 완료했다면
    //         setDays(data)
    //     })
    // }, [])

    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>


     </>;
}