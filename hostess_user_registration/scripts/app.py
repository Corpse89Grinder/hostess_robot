#!/usr/bin/python
from flask import Flask, render_template, request, redirect, url_for
from flask.ext.wtf import Form
from wtforms import StringField, IntegerField, SelectField, SubmitField, FloatField
from wtforms.validators import Required

from flask_bootstrap import Bootstrap
from flask.ext.sqlalchemy import SQLAlchemy
from sqlalchemy import func
import os, sys
from flask_wtf import form
from matplotlib.pyplot import connect
from sqlalchemy.sql.expression import desc
from sqlalchemy.sql.schema import UniqueConstraint
from dbus.decorators import method
from types import NoneType
from numpy import integer, int
from sqlalchemy.sql.elements import False_
from flask.templating import render_template

basedir = os.path.abspath(os.path.dirname(__file__))

from flask.ext.script import Manager 
from flask.ext.migrate import Migrate, MigrateCommand

script = sys.argv[0]
command = sys.argv[1]

if command == 'db':
    sys.argv = script, command, sys.argv[2]
elif command == 'runserver':
    sys.argv = script, command, '-d'

app = Flask(__name__)
app.config['SECRET_KEY'] = 'hard to guess string'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'data.sqlite')
app.config['SQLALCHEMY_COMMIT_ON_TEARDOWN'] = True
db = SQLAlchemy(app)
Bootstrap(app)
manager = Manager(app)
migrate = Migrate(app, db)
manager.add_command('db', MigrateCommand)


class AddNewUserIndexForm(Form):
    add_new_user = SubmitField('Aggiungi nuovo utente')
    
class AddNewGoalIndexForm(Form):
    add_new_goal = SubmitField('Aggiungi nuovo goal')

class RegistrationForm(Form):
    name = StringField('Nome')
    surname = StringField('Cognome')
    mail = StringField('Email')
    goal = SelectField('Dove vuoi andare', choices=[], coerce=int)
    submit = SubmitField('Invia')

class AddGoalForm(Form):
    label = StringField('label')
    x = FloatField('x')
    y = FloatField('y')
    submit = SubmitField('Invia')


class CheckInForm(Form):
    mail = StringField('Email')
    submit = SubmitField('Invia')


class User(db.Model):
    __tablename__ = 'users'
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(64))
    surname = db.Column(db.String(64))
    goal_id = db.Column(db.Integer, db.ForeignKey('goals.id'))
    email = db.Column(db.String(64), unique=True, index=True)
    calibrated = db.Column(db.Integer)

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


@app.route('/users')
def users():
    return render_template('users.html', users=User.query.all())

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
    return render_template('register.html', form=form)


@app.route('/new_user', methods=['GET', 'POST'])
def new_user():
    form = RegistrationForm()
    form.goal.choices = [(g.id, g.label) for g in Goal.query.all()]
    return render_template('register.html', form=form)
    
@app.route('/new_user/calibration', methods=['POST'])
def calibration():
    form = RegistrationForm(request.form)
    goal = Goal.query.filter_by(id=form.goal.data).first_or_404()
    return render_template('user_calibration.html', form=form, goal=goal)

@app.route('/checkin', methods=['GET', 'POST'])
def checkin():
    form = CheckInForm(request.form)
    if request.method == 'POST' and form.validate():
        user = User.query.filter_by(email=form.mail.data).first_or_404()
        if user is not None:
            return render_template('checkin.html', user=user)
    return render_template('register.html', form=form)


@app.route('/')
def root():
    return redirect(url_for('.index'))

@app.route('/index', methods=['GET', 'POST'])
def index():
    new_user_form = AddNewUserIndexForm(request.form)
    new_goal_form = AddNewGoalIndexForm(request.form)
    if request.method == 'POST':
        if 'add_new_user' in request.form:
            return redirect(url_for('.new_user'))
        elif 'add_new_goal' in request.form:
            return redirect(url_for('.new_goal'))
    return render_template('index.html', new_user_form=new_user_form, new_goal_form=new_goal_form)


if __name__ == '__main__':
    manager.run()	
