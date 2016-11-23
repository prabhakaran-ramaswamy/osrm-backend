import 'babel-polyfill';
import React from 'react';
import ReactDOM from 'react-dom';
import App from './components/app';
import remark from 'remark';
import slug from 'remark-slug';

var fs = require('fs');

var content = fs.readFileSync('../docs/http.md','utf8');

var ast = remark()
  .use(slug)
  .run(remark().parse(content));

ReactDOM.render(
      <App ast={ast} content={content} />,
      document.getElementById('app'));
