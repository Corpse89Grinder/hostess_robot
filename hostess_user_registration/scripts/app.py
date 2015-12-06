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
sys.argv = [script, 'runserver', '--host', '0.0.0.0']

print sys.argv

app = Flask(__name__)
app.config['SECRET_KEY'] = 'hard to guess string'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'data.sqlite')
app.config['SQLALCHEMY_COMMIT_ON_TEARDOWN'] = True
db = SQLAlchemy(app)
Bootstrap(app)
manager = Manager(app)
migrate = Migrate(app, db)
manager.add_command('db', MigrateCommand)

class RegistrationForm(Form):
    name = StringField('Nome')
    surname = StringField('Cognome')
    mail = StringField('E-Mail')
    goal = SelectField('Dove vuoi andare?', choices=[], coerce=int)

class AddGoalForm(Form):
    label = StringField('Destinazione')
    x = FloatField('X')
    y = FloatField('Y')

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
    else:
        print 'arrivata'
    return render_template('new_goal.html', form=form)

@app.route('/delete_goals')
def delete_goal():
    return render_template('delete_goals.html', goals=Goal.query.all())

@app.route('/users')
def users():
    return render_template('users.html', users=User.query.all())

@app.route('/new_user')
def new_user():
    form = RegistrationForm()
    form.goal.choices = [(g.id, g.label) for g in Goal.query.all()]
    return render_template('new_user.html', form=form)
    
@app.route('/new_user/calibration', methods=['POST'])
def calibration():
    form = RegistrationForm(request.form)
    goal = Goal.query.filter_by(id=form.goal.data).first_or_404()
    return render_template('user_calibration.html', form=form, goal=goal)

@app.route('/delete_users')
def delete_user():
    return render_template('delete_users.html', users=User.query.all())

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

@app.route('/index')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    manager.run()