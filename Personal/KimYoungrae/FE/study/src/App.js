
import './App.css';
import Header from './component/Header'
import DayList from './component/DayList';
import Day from './component/Day';
import EmptyPage from './component/EmptyPage';
import CreateWord from './component/CreateWord';
import CreateDay from './component/CreateDay';
import {BrowserRouter, Route, Switch} from 'react-router-dom';


function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Header />
        <Switch>
          <Route exact path="/">
            <DayList />
          </Route>
          
          <Route path="/day/:day">
            <Day />
          </Route>

          <Route path="/create_word">
            <CreateWord />
          </Route>

          <Route path="/create_day">
            <CreateDay />
          </Route>

          <Route> 
            {/* Route안에 path가 없으면 가장 조건으로 접근함...  주의! 이건 항상 가장 마지막에 해야함*/}
            <EmptyPage />
          </Route>
        </Switch>
      </div>
    </BrowserRouter>
    
  );
}

export default App;
