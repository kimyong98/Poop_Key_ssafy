
// get하는 녀석

import { useEffect, useState } from "react"

export default function useFetch(url) {
    

    const [data, setData] = useState([]);

    useEffect(()=>{
        fetch(url)
        // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
        .then(res=>{
            // fetch가 되면 == .then
            return res.json()
            // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
            //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
        })
        .then(data=>{
            // 위에걸 완료했다면
            setData(data)
        })
    }, [url]);

    return data;
}