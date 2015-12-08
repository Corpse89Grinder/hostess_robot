#!/usr/bin/python
import eventlet

eventlet.monkey_patch()

import os, sys, json, roslib, rospy, actionlib
from flask import Flask, render_template, request, redirect, url_for
from flask.ext.wtf import Form
from wtforms import StringField, IntegerField, SelectField, SubmitField, FloatField, validators
from wtforms.validators import Required
from flask_bootstrap import Bootstrap
from flask.ext.sqlalchemy import SQLAlchemy
from sqlalchemy import func
from flask_wtf import form
from matplotlib.pyplot import connect
from sqlalchemy.sql.schema import UniqueConstraint
from dbus.decorators import method
from numpy import integer, int
from flask.templating import render_template
from flask.ext.migrate import Migrate, MigrateCommand
from cob_people_detection.msg import addDataAction, addDataGoal, deleteDataAction, deleteDataGoal
from flask_socketio import SocketIO, emit, disconnect

basedir = os.path.abspath(os.path.dirname(__file__))

app = Flask(__name__)
app.config['SECRET_KEY'] = 'hard to guess string'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'data.sqlite')
app.config['SQLALCHEMY_COMMIT_ON_TEARDOWN'] = True
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = True
db = SQLAlchemy(app)
Bootstrap(app)
migrate = Migrate(app, db)
socketio = SocketIO(app, async_mode='eventlet')

class User(db.Model):
    __tablename__ = 'users'
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(64))
    surname = db.Column(db.String(64))
    goal_id = db.Column(db.Integer, db.ForeignKey('goals.id'))
    email = db.Column(db.String(64), unique=True, index=True)

    def __repr__(self):
        return '<User %r>' % self.name

class Goal(db.Model):
    __tablename__ = 'goals'
    id = db.Column(db.Integer, primary_key=True)
    label = db.Column(db.String(64))
    x = db.Column(db.Float)
    y = db.Column(db.Float)
    users = db.relationship('User', backref='goal')

    def __repr__(self):
        return '<User %r>' % self.label

class RegistrationForm(Form):
    name = StringField('Nome')
    surname = StringField('Cognome')
    mail = StringField('E-Mail')
    goal = SelectField('Dove vuoi andare?', choices=[(g.id, g.label) for g in Goal.query.all()], coerce=int)
    
    def __init__(self, *args, **kwargs):
        Form.__init__(self, *args, **kwargs)
    
    def validate(self):
        outcome = True
        
        rv = Form.validate(self)
        if not rv:
            outcome = False
        
        if self.name.data == '':
            self.name.errors.append('Campo obbligatorio')
            outcome = False
            
        if self.surname.data == '':
            self.surname.errors.append('Campo obbligatorio')
            outcome = False
            
        if self.mail.data == '':
            self.mail.errors.append('Campo obbligatorio')
            outcome = False
        else:
            mail = User.query.filter_by(email=self.mail.data).first()
            if mail is not None:
                self.mail.errors.append('E-Mail presente in database, inserirne una differente')
                outcome = False
        
        found = False
        for v, _ in self.goal.choices:
            if self.goal.data == v:
                found = True
                break
        
        if found == False:
            self.goal.errors.append('Seleziona una destinazione')

        return outcome


class AddGoalForm(Form):
    label = StringField('Destinazione', [validators.Required()])
    x = FloatField('X')
    y = FloatField('Y')

@app.route('/goals')
def goals():
    return render_template('goals.html', goals=Goal.query.all())

@app.route('/new_goal', methods=['GET', 'POST'])
def new_goal():
    form = AddGoalForm(request.form)
    if request.method == 'POST' and form.validate():
        goal = Goal(label=form.label.data, x=form.x.data, y=form.y.data)
        db.session.add(goal)
        db.session.commit()
        return redirect(url_for('.goals'))
    return render_template('new_goal.html', form=form)

@app.route('/delete_goals', methods=['GET', 'POST'])
def delete_goal():
    return render_template('delete_goals.html', goals=Goal.query.all())

@app.route('/users')
def users():
    return render_template('users.html', users=User.query.all(), goals=len(Goal.query.all()))

@app.route('/new_user', methods=['GET', 'POST'])
def new_user():
    form = RegistrationForm()
    select = 0
    if request.method == 'POST':
        if form.validate():
            return redirect(url_for('.user_calibration'), 307)
        else:
            if form.goal.data is not None:
                select = form.goal.data
    return render_template('new_user.html', form=form, select=select)

@app.route('/new_user/calibration', methods=['POST'])
def user_calibration():
    if request.method == 'POST':
        form = RegistrationForm(request.form)
        goal = Goal.query.filter_by(id=form.goal.data).first_or_404()
        return render_template('user_calibration.html', form=form, goal=goal)
    
@socketio.on('start', namespace='/new_user/calibration')
def start_calibration():
    id = db.session.query(db.func.max(User.id)).scalar()
    if id is None:
        id = 1
    else:
        id = id + 1
        
    id_string = '00000000' + str(id)
    id_string = id_string[-8:]
        
    client = actionlib.SimpleActionClient('add_user', addDataAction)
    client.wait_for_server()
    
    goal = addDataGoal(label=id_string, capture_mode=1, continuous_mode_images_to_capture=100, continuous_mode_delay=0.03)
    
    client.send_goal(goal, done_cb, active_cb, feedback_cb)
    
def done_cb(state, result):
    socketio.emit('calibrated', namespace='/new_user/calibration')
    
def active_cb():
    socketio.emit('started', namespace='/new_user/calibration')
    
def feedback_cb(feedback):
    socketio.emit('progress', {'images_captured': feedback.images_captured}, namespace='/new_user/calibration')

@socketio.on('credentials', namespace='/new_user/calibration')
def save_user(message):
    name=message['nome']
    surname=message['cognome']
    goal_id=message['destinazione']
    email=message['email']
    
    user = User(name=name, surname=surname, goal_id=goal_id, email=email)
    db.session.add(user)
    db.session.commit()

    emit('saved')
    disconnect()
    
@app.route('/delete_users')
def delete_user():
    return render_template('delete_users.html', users=User.query.all())

@app.route('/delete_user_entries', methods=['POST'])
def delete_user_entries():
    if request.method == 'POST' and request.headers['Content-Type'] == 'application/json; charset=UTF-8':
        client = actionlib.SimpleActionClient('delete_user', deleteDataAction)
        client.wait_for_server()
        
        message = json.loads(request.data)
        
        for i in message['utenti']:
            id_string = '00000000' + str(i)
            id_string = id_string[-8:]
            
            goal=deleteDataGoal(delete_mode=2, label=id_string)
            
            client.send_goal(goal)
            client.wait_for_result()
            
            user = User.query.filter_by(id=i).first_or_404()
            db.session.delete(user)
            
        db.session.commit()
        return 'Ok'
    else:
        return 'Bad'
    
@app.route('/delete_goal_entries', methods=['POST'])
def delete_goal_entries():
    print request.headers['Content-Type']
    if request.method == 'POST' and request.headers['Content-Type'] == 'application/json; charset=UTF-8':
        message = json.loads(request.data)
        for i in message['destinazioni']:
            goal = Goal.query.filter_by(id=i).first_or_404()
            db.session.delete(goal)
            db.session.commit()
        return 'Ok'
    else:
        return 'Bad'

@app.route('/')
def root():
    return redirect(url_for('.index'))

@app.route('/index')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    rospy.init_node('hostess_management')
    socketio.run(app, debug=True, host='0.0.0.0')